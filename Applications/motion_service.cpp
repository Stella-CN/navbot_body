#include "motion_service.hpp"
#include "stm32f4xx_hal.h"
#include "wheel_motors.hpp"
#include "c30d_safety.h"
#include "link_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cmath>
#include <cstring>
extern "C"
{
    volatile uint32_t navbot_motion_max_cycles;
    volatile uint32_t navbot_motion_recoveries[4];
}
namespace
{
    navbot::Motion *controller;
    struct Request
    {
        uint8_t id{};
        uint8_t bytes[18]{};
        uint32_t seq{}, time{};
        bool pending{};
    };
    Request mailbox;
    MotionServiceSnapshot published;
    float number(const uint8_t *p)
    {
        uint32_t bits = static_cast<uint32_t>(Link_Get(p, 4));
        float x;
        std::memcpy(&x, &bits, 4);
        return x;
    }
}
void Navbot_MotionInit()
{
    auto &w = navbot::WheelMotors::instance();
    static navbot::Motion instance({&w.frontLeft(), &w.rearLeft(), &w.frontRight(), &w.rearRight()});
    controller = &instance;
    // A stopped control task cannot leave PWM running indefinitely.
    DBGMCU->APB1FZ &= ~DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR = 0xCCCC;
    IWDG->KR = 0x5555;
    IWDG->PR = 3;
    IWDG->RLR = 499;
    uint32_t start = HAL_GetTick();
    while (IWDG->SR)
        if (HAL_GetTick() - start > 50)
            C30D_TemplatePanic();
    IWDG->KR = 0xAAAA;
}
void Navbot_MotionStop(uint32_t seq)
{
    taskENTER_CRITICAL();
    mailbox = {};
    mailbox.id = 0x12;
    mailbox.seq = seq;
    mailbox.pending = true;
    C30D_ForceMotorIdle();
    taskEXIT_CRITICAL();
}
uint8_t Navbot_MotionRequest(uint8_t id, const uint8_t *p, uint16_t n, uint32_t seq, uint32_t now)
{
    uint16_t expected = id == 0x11 ? 1 : id == 0x10 ? 10
                                     : id == 0x14   ? 18
                                     : id == 0x31   ? 12
                                                    : 0xffff;
    if (n != expected)
        return 2;
    if (id == 0x11 && p[0] > 1)
        return 5;
    if (id == 0x11 && !p[0])
    {
        Navbot_MotionStop(seq);
        return 0;
    }
    if (id == 0x10 || id == 0x14)
    {
        unsigned count = id == 0x10 ? 2 : 4;
        for (unsigned i = 0; i < count; ++i)
            if (!std::isfinite(number(p + 4 * i)))
                return 5;
        if (id == 0x10 && (std::fabs(number(p)) > .3f || std::fabs(number(p + 4)) > 1.5f))
            return 5;
        if (id == 0x14)
            for (unsigned i = 0; i < 4; ++i)
                if (std::fabs(number(p + 4 * i)) > 12)
                    return 5;
        uint64_t ttl = Link_Get(p + 4 * count, 2);
        if (ttl < 50 || ttl > 250)
            return 5;
    }
    if (id == 0x31)
    {
        float r = number(p), b = number(p + 4);
        if (!std::isfinite(r) || !std::isfinite(b) || r < .02f || r > .2f || b < .1f || b > 1)
            return 5;
        for (unsigned i = 8; i < 12; ++i)
            if (p[i] != 1 && p[i] != 255)
                return 5;
    }
    taskENTER_CRITICAL();
    auto s = published.motion;
    uint8_t result = 0;
    bool moving = id == 0x10 || id == 0x14;
    if ((moving && s.state != navbot::MotionState::Enabled) || (!moving && s.state != navbot::MotionState::Disabled) || s.fault || (id == 0x10 && !s.configured))
        result = 4;
    else if (mailbox.pending && !(moving && (mailbox.id == 0x10 || mailbox.id == 0x14)))
        result = 6;
    else
    {
        mailbox.id = id;
        std::memcpy(mailbox.bytes, p, n);
        mailbox.seq = seq;
        mailbox.time = now;
        mailbox.pending = true;
    }
    taskEXIT_CRITICAL();
    return result;
}
void Navbot_MotionPoll(uint32_t now)
{
    // Bounded four-wheel calculation/register writes, no blocking or allocation.
    // STOP's direct register cut-off cannot interleave with and be undone by PWM writes.
    uint32_t start_cycles = DWT->CYCCNT;
    taskENTER_CRITICAL();
    // Consume a latched STOP before running the old controller target. Otherwise
    // update() could briefly reassert PWM after the communication task cut it off.
    if (mailbox.pending && mailbox.id == 0x12)
    {
        controller->stop(controller->snapshot().fault ? navbot::MotionState::Fault : navbot::MotionState::Disabled);
        published.request_seq = mailbox.seq;
        published.result = 0;
        mailbox.pending = false;
    }
    controller->update(now);
    if (mailbox.pending)
    {
        Request r = mailbox;
        mailbox.pending = false;
        bool ok = false;
        if (r.id == 0x11)
            ok = controller->enable(now);
        else if (r.id == 0x31)
        {
            int8_t signs[4];
            for (unsigned i = 0; i < 4; ++i)
                signs[i] = r.bytes[8 + i] == 1 ? 1 : -1;
            ok = controller->configure(number(r.bytes), number(r.bytes + 4), signs);
        }
        else
        {
            unsigned count = r.id == 0x10 ? 2 : 4;
            uint16_t ttl = static_cast<uint16_t>(Link_Get(r.bytes + 4 * count, 2));
            if (now - r.time < ttl)
            {
                // Preserve receipt deadline; processing never extends a stale command.
                uint16_t remaining = ttl - static_cast<uint16_t>(now - r.time);
                if (remaining >= 50)
                {
                    if (r.id == 0x10)
                        ok = controller->body(number(r.bytes), number(r.bytes + 4), remaining, r.seq, now);
                    else
                    {
                        float v[4];
                        for (unsigned i = 0; i < 4; ++i)
                            v[i] = number(r.bytes + 4 * i);
                        ok = controller->wheels(v, remaining, r.seq, now);
                    }
                }
            }
        }
        published.request_seq = r.seq;
        published.result = ok ? 0 : 4;
    }
    published.motion = controller->snapshot();
    for (unsigned i = 0; i < 4; ++i)
        navbot_motion_recoveries[i] = published.motion.recoveries[i];
    taskEXIT_CRITICAL();
    uint32_t cycles = DWT->CYCCNT - start_cycles;
    if (cycles > navbot_motion_max_cycles)
        navbot_motion_max_cycles = cycles;
    DBGMCU->APB1FZ &= ~DBGMCU_APB1_FZ_DBG_IWDG_STOP;
    IWDG->KR = 0xAAAA;
}
MotionServiceSnapshot Navbot_MotionSnapshot()
{
    taskENTER_CRITICAL();
    auto result = published;
    taskEXIT_CRITICAL();
    return result;
}
