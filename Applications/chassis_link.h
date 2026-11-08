#ifndef NAVBOT_CHASSIS_LINK_H
#define NAVBOT_CHASSIS_LINK_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>
    void Navbot_LinkInit(void);
    void Navbot_LinkPoll(uint32_t now_ms);
#ifdef __cplusplus
}
#endif
#endif
