#include "motion_service.hpp"
#include "wheel_motors.hpp"
#include "link_protocol.h"
#include "stm32f4xx_hal.h"
#include <cassert>
#include <cstdlib>
#include <cstdio>
MockDebug mock_debug;
MockWatchdog mock_watchdog;
MockDwt mock_dwt;
int critical_depth;
unsigned writes;
int32_t outputs[4];
extern "C"
{
    int C30D_MotorInit(C30D_MotorChannel c)
    {
        outputs[c] = 0;
        return 1;
    }
    int C30D_MotorWrite(C30D_MotorChannel c, int32_t d)
    {
        assert(critical_depth > 0);
        outputs[c] = d;
        ++writes;
        return 1;
    }
    void C30D_MotorCoast(C30D_MotorChannel c) { outputs[c] = 0; }
    int C30D_MotorBrake(C30D_MotorChannel) { return 1; }
    uint32_t C30D_MotorCounter(C30D_MotorChannel) { return 0; }
    void C30D_ForceMotorIdle()
    {
        for (auto &o : outputs)
            o = 0;
    }
    [[noreturn]] void C30D_TemplatePanic() { std::abort(); }
}
int main()
{
    auto &w = navbot::WheelMotors::instance();
    assert(w.initialize(0));
    Navbot_MotionInit();
    uint32_t now = 0;
    auto tick = [&]()
    {now+=10;assert(w.updateFeedback(now));Navbot_MotionPoll(now);assert(critical_depth==0); };
    tick();
    uint8_t enabled = 1;
    assert(Navbot_MotionRequest(0x11, &enabled, 1, 1, now) == 0);
    tick();
    uint8_t v[18]{};
    Link_Float(v, 6);
    Link_Put(v + 16, 200, 2);
    assert(Navbot_MotionRequest(0x14, v, 18, 2, now) == 0);
    tick();
    tick();
    assert(writes > 0 && outputs[C30D_MOTOR_C] > 0);
    auto before = writes;
    Navbot_MotionStop(3);
    assert(outputs[C30D_MOTOR_C] == 0);
    assert(Navbot_MotionRequest(0x14, v, 18, 4, now) == 6); // STOP cannot be overwritten.
    tick();
    assert(writes == before); // Crucial: even transient reassertion is forbidden.
    assert(Navbot_MotionSnapshot().motion.state == navbot::MotionState::Disabled);
    assert(Navbot_MotionSnapshot().request_seq == 3);
    assert(Navbot_MotionRequest(0x11, &enabled, 1, 5, now) == 0);
    assert(Navbot_MotionRequest(0x11, &enabled, 1, 6, now) == 6);
    tick();
    assert(Navbot_MotionRequest(0x14, v, 18, 7, now) == 0);
    Link_Float(v, 3);
    assert(Navbot_MotionRequest(0x14, v, 18, 8, now) == 0);
    tick();
    assert(Navbot_MotionSnapshot().motion.applied_seq == 8);
    assert(Navbot_MotionSnapshot().motion.target[0] == 3);
    for (int i = 0; i < 20; ++i)
        tick();
    assert(Navbot_MotionSnapshot().motion.state == navbot::MotionState::Timeout);
    assert(Navbot_MotionRequest(0x11, &enabled, 1, 9, now) == 4);
    Navbot_MotionStop(10);
    tick();
    assert(Navbot_MotionSnapshot().motion.state == navbot::MotionState::Disabled);
    puts("motion service: STOP never reasserts PWM, mailbox priority/latest-wins, execution ACK and timeout passed");
}
