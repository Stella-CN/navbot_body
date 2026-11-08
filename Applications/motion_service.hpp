#ifndef NAVBOT_MOTION_SERVICE_HPP
#define NAVBOT_MOTION_SERVICE_HPP
#include "motion.hpp"
struct MotionServiceSnapshot
{
    navbot::MotionSnapshot motion;
    uint32_t request_seq{}, result{};
};
void Navbot_MotionInit();
void Navbot_MotionPoll(uint32_t now);
// Comm-task mailbox API. Result 0 means queued, not executed/reached target.
uint8_t Navbot_MotionRequest(uint8_t id, const uint8_t *payload, uint16_t length, uint32_t seq, uint32_t now);
void Navbot_MotionStop(uint32_t seq);
MotionServiceSnapshot Navbot_MotionSnapshot();
#endif
