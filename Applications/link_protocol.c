#include "link_protocol.h"
#include <string.h>
uint64_t Link_Get(const uint8_t *p, unsigned n)
{
    uint64_t value = 0;
    for (unsigned i = 0; i < n; ++i)
        value |= (uint64_t)p[i] << (8U * i);
    return value;
}
void Link_Put(uint8_t *p, uint64_t value, unsigned n)
{
    for (unsigned i = 0; i < n; ++i)
        p[i] = (uint8_t)(value >> (8U * i));
}
void Link_Float(uint8_t *p, float value)
{
    uint32_t bits;
    _Static_assert(sizeof(float) == 4, "float32 required");
    memcpy(&bits, &value, sizeof(bits));
    Link_Put(p, bits, 4);
}
uint16_t Link_Crc(const uint8_t *p, size_t n)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < n; ++i)
    {
        crc ^= (uint16_t)p[i] << 8;
        for (unsigned j = 0; j < 8; ++j)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
    }
    return crc;
}
size_t Link_Encode(uint8_t *out, const LinkFrame *f)
{
    if (f->length > LINK_MAX_PAYLOAD)
        return 0;
    out[0] = 0x5a;
    out[1] = 0xa5;
    out[2] = 1;
    out[3] = f->id;
    Link_Put(out + 4, f->length, 2);
    Link_Put(out + 6, f->sequence, 4);
    Link_Put(out + 10, f->session, 4);
    Link_Put(out + 14, f->timestamp_us, 8);
    memcpy(out + 22, f->payload, f->length);
    Link_Put(out + 22 + f->length, Link_Crc(out + 2, 20U + f->length), 2);
    return 24U + f->length;
}
static void discard(LinkParser *p, size_t n, uint32_t now)
{
    p->used -= n;
    memmove(p->bytes, p->bytes + n, p->used);
    p->started_ms = now;
}
void Link_Feed(LinkParser *p, uint8_t byte, uint32_t now)
{
    if (p->used == sizeof(p->bytes))
        discard(p, 1, now);
    if (p->used == 0)
        p->started_ms = now;
    p->bytes[p->used++] = byte;
}
int Link_Next(LinkParser *p, LinkFrame *f, uint32_t now)
{
    while (p->used)
    {
        if (p->bytes[0] != 0x5a || (p->used > 1 && p->bytes[1] != 0xa5))
        {
            discard(p, 1, now);
            continue;
        }
        if ((uint32_t)(now - p->started_ms) >= 20U)
        {
            ++p->timeouts;
            discard(p, 1, now);
            continue;
        }
        if (p->used < 6)
            return 0;
        if (p->bytes[2] != 1)
        {
            ++p->version_errors;
            discard(p, 1, now);
            continue;
        }
        size_t len = (size_t)Link_Get(p->bytes + 4, 2);
        if (len > LINK_MAX_PAYLOAD)
        {
            ++p->length_errors;
            discard(p, 1, now);
            continue;
        }
        if (p->used < len + 24U)
            return 0;
        if (Link_Crc(p->bytes + 2, len + 20U) != Link_Get(p->bytes + 22 + len, 2))
        {
            ++p->crc_errors;
            discard(p, 1, now);
            continue;
        }
        f->id = p->bytes[3];
        f->length = (uint16_t)len;
        f->sequence = (uint32_t)Link_Get(p->bytes + 6, 4);
        f->session = (uint32_t)Link_Get(p->bytes + 10, 4);
        f->timestamp_us = Link_Get(p->bytes + 14, 8);
        memcpy(f->payload, p->bytes + 22, len);
        discard(p, len + 24U, now);
        return 1;
    }
    return 0;
}
