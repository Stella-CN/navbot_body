#ifndef NAVBOT_LINK_PROTOCOL_H
#define NAVBOT_LINK_PROTOCOL_H
#include <stddef.h>
#include <stdint.h>
#define LINK_MAX_PAYLOAD 256U
#define LINK_MAX_FRAME 280U
typedef struct
{
    uint8_t bytes[LINK_MAX_FRAME];
    size_t used;
    uint32_t crc_errors, length_errors, version_errors, timeouts;
    uint32_t started_ms;
} LinkParser;
typedef struct
{
    uint8_t id;
    uint16_t length;
    uint32_t sequence, session;
    uint64_t timestamp_us;
    uint8_t payload[LINK_MAX_PAYLOAD];
} LinkFrame;
/* Internal field helpers: n is in [1, 8]; callers provide that many bytes. */
uint64_t Link_Get(const uint8_t *p, unsigned n);
void Link_Put(uint8_t *p, uint64_t value, unsigned n);
void Link_Float(uint8_t *p, float value);
uint16_t Link_Crc(const uint8_t *p, size_t n);
size_t Link_Encode(uint8_t *out, const LinkFrame *frame);
/* Single owner. Feed one byte then drain Next; also call Next on idle for timeout. */
void Link_Feed(LinkParser *p, uint8_t byte, uint32_t now_ms);
int Link_Next(LinkParser *p, LinkFrame *frame, uint32_t now_ms);
#endif
