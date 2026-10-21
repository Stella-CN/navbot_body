#ifndef NAVBOT_IMU_DIAGNOSTICS_H
#define NAVBOT_IMU_DIAGNOSTICS_H
#include "c30d_imu.h"
#include "attitude.h"
/* Short critical sections copy one fixed-size value; no pointers escape. */
void Navbot_ImuPublish(const C30D_ImuSnapshot *snapshot, const Navbot_AttitudeSnapshot *attitude);
void Navbot_ImuSnapshot(C30D_ImuSnapshot *snapshot, Navbot_AttitudeSnapshot *attitude);
void Navbot_ImuDisplay(uint32_t now_ms);
extern uint32_t navbot_oled_frames, navbot_oled_max_us;
extern uint32_t navbot_stack_free_words[4];
#endif
