#ifndef NAVBOT_MOTION_HPP
#define NAVBOT_MOTION_HPP
#include "motor.hpp"
#include <array>
namespace navbot
{
    // Wire order LF, LR, RF, RR. All storage is owned, except fixed motor references.
    enum class MotionState : uint32_t
    {
        Disabled,
        Enabled,
        Timeout,
        Fault
    };
    struct MotionSnapshot
    {
        MotorFeedback feedback[4]{};
        float target[4]{}, ramp[4]{};
        int32_t duty[4]{};
        MotionState state{};
        uint32_t recoveries[4]{};
        uint32_t timestamp{}, applied_seq{}, command_age{}, fault{}, valid{}, configured{};
    };
    class Motion
    {
    public:
        explicit Motion(std::array<Motor *, 4> motors) : motors_(motors) {}
        bool enable(uint32_t now);
        void stop(MotionState state = MotionState::Disabled);
        bool configure(float radius, float track, const int8_t signs[4]);
        bool wheels(const float omega[4], uint16_t ttl, uint32_t seq, uint32_t now);
        bool body(float vx, float wz, uint16_t ttl, uint32_t seq, uint32_t now);
        void update(uint32_t now);
        MotionSnapshot snapshot() const { return data_; }

    private:
        std::array<Motor *, 4> motors_;
        MotionSnapshot data_{};
        float integral_[4]{}, filtered_[4]{}, radius_{}, track_{};
        uint16_t starting_ms_[4]{}, moving_ms_[4]{}, stalled_ms_[4]{};
        bool started_[4]{};
        int8_t signs_[4]{1, 1, 1, 1};
        uint32_t last_update_{}, command_time_{};
        uint16_t ttl_{200};
    };
}
#endif
