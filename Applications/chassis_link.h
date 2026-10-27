#ifndef NAVBOT_CHASSIS_LINK_H
#define NAVBOT_CHASSIS_LINK_H
#include <stdint.h>
void Navbot_LinkInit(void);
void Navbot_LinkPoll(uint32_t now_ms);
#endif
