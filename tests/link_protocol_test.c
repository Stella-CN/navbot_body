#include "link_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static LinkParser parser;
static LinkFrame input, output;
static uint8_t packet[LINK_MAX_FRAME];
static int feed(const uint8_t *data, size_t length, uint32_t now)
{
    int frames = 0;
    for (size_t i = 0; i < length; ++i) {
        Link_Feed(&parser, data[i], now);
        while (Link_Next(&parser, &output, now)) ++frames;
        assert(parser.used <= LINK_MAX_FRAME);
    }
    return frames;
}
int main(void)
{
    assert(Link_Crc((const uint8_t *)"123456789", 9) == 0x29b1);
    input.id = 0x12; input.sequence = 0x12345678; input.session = 0xabcdef01;
    input.timestamp_us = 0x0102030405060708ULL;
    const uint8_t golden[] = {0x5a,0xa5,1,0x12,0,0,0x78,0x56,0x34,0x12,1,0xef,0xcd,0xab,
                            8,7,6,5,4,3,2,1,0x94,0x8e};
    assert(Link_Encode(packet, &input) == sizeof(golden));
    assert(memcmp(packet, golden, sizeof(golden)) == 0);
    assert(feed(packet, sizeof(golden), 0) == 1);
    assert(output.sequence == input.sequence && output.session == input.session && output.timestamp_us == input.timestamp_us);
    for (unsigned length = 0; length <= 256; ++length) {
        input.length = length;
        for (unsigned i = 0; i < length; ++i) input.payload[i] = (uint8_t)i;
        size_t n = Link_Encode(packet, &input);
        assert(feed(packet, n, length) == 1);
        assert(output.length == length && !memcmp(input.payload, output.payload, length));
    }
    input.length = 257;
    assert(Link_Encode(packet, &input) == 0);
    input.length = 0;
    size_t n = Link_Encode(packet, &input);
    packet[n-1] ^= 1;
    assert(feed(packet, n, 300) == 0);
    assert(parser.crc_errors > 0);
    const uint8_t invalid[] = {0x5a,0xa5,1,0,0xff,0xff,0x5a,0xa5,2,0,0,0};
    assert(feed(invalid, sizeof(invalid), 301) == 0);
    assert(parser.length_errors && parser.version_errors);
    n = Link_Encode(packet, &input);
    assert(feed(packet, 8, 0xfffffff0U) == 0);
    assert(!Link_Next(&parser, &output, 5)); /* wrap-safe timeout */
    assert(parser.timeouts);
    assert(feed(packet, n, 6) == 1);
    assert(feed(packet, n, 6) == 1); /* concatenated frames */
    uint32_t random = 1;
    for (unsigned i = 0; i < 100000; ++i) {
        random = random * 1664525U + 1013904223U;
        uint8_t b = random >> 24;
        (void)feed(&b, 1, i / 32);
    }
    /* Drain hostile partial headers before a final known frame. */
    for (unsigned i = 0; i < 281; ++i) (void)Link_Next(&parser, &output, 4000 + i * 21);
    assert(feed(packet, n, 10000) == 1);
    puts("PASS: CRC/golden vector, all payload lengths, fragmentation/coalescing, CRC/length/version, timeout/tick wrap, 100k noise bytes");
}
