#include "motion.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace navbot
{
    namespace
    {
        struct WheelTuning
        {
            float friction, velocity, kp, ki, breakaway, filter_seconds;
        };
        // LF(C), LR(D), RF(B), RR(A); output in permille, speed in rad/s.
        // Bench estimates, not loaded/temperature-independent motor constants.
        constexpr WheelTuning tuning[4] = {
            {45, 9, 40, 60, 90, .002f}, {20, 9, 160, 60, 135, .0005f}, {30, 9, 40, 60, 85, .002f}, {15, 8, 160, 60, 140, .0005f}};
        constexpr float max_speed = 12.0f, max_duty = 200.0f, acceleration = 20.0f;
    }
    void Motion::stop(MotionState state)
    {
        data_.state = state;
        for (unsigned i = 0; i < 4; ++i)
        {
            motors_[i]->disable();
            integral_[i] = filtered_[i] = 0;
            starting_ms_[i] = moving_ms_[i] = stalled_ms_[i] = 0;
            started_[i] = false;
            data_.target[i] = data_.ramp[i] = 0;
            data_.duty[i] = 0;
        }
    }
    bool Motion::enable(uint32_t now)
    {
        if (data_.state != MotionState::Disabled || data_.fault || data_.valid != 15 || now - data_.timestamp > 20)
            return false;
        for (auto *m : motors_)
            if (!m->enable())
            {
                data_.fault = 1;
                stop(MotionState::Fault);
                return false;
            }
        for (unsigned i = 0; i < 4; ++i)
            data_.recoveries[i] = 0;
        command_time_ = now;
        ttl_ = 200;
        data_.state = MotionState::Enabled;
        return true;
    }
    bool Motion::configure(float radius, float track, const int8_t signs[4])
    {
        if (data_.state != MotionState::Disabled || !std::isfinite(radius) || !std::isfinite(track) || radius < .02f || radius > .2f || track < .1f || track > 1.0f)
            return false;
        for (unsigned i = 0; i < 4; ++i)
            if (signs[i] != 1 && signs[i] != -1)
                return false;
        radius_ = radius;
        track_ = track;
        for (unsigned i = 0; i < 4; ++i)
            signs_[i] = signs[i];
        data_.configured = 1;
        return true;
    }
    bool Motion::wheels(const float omega[4], uint16_t ttl, uint32_t seq, uint32_t now)
    {
        if (data_.state != MotionState::Enabled || ttl < 50 || ttl > 250)
            return false;
        // Expired commands cannot be revived by a packet arriving before the next tick.
        if (now - command_time_ >= ttl_)
        {
            stop(MotionState::Timeout);
            return false;
        }
        for (unsigned i = 0; i < 4; ++i)
            if (!std::isfinite(omega[i]) || std::fabs(omega[i]) > max_speed)
                return false;
        for (unsigned i = 0; i < 4; ++i)
            data_.target[i] = omega[i];
        command_time_ = now;
        ttl_ = ttl;
        data_.applied_seq = seq;
        return true;
    }
    bool Motion::body(float vx, float wz, uint16_t ttl, uint32_t seq, uint32_t now)
    {
        if (!data_.configured || !std::isfinite(vx) || !std::isfinite(wz) || std::fabs(vx) > .3f || std::fabs(wz) > 1.5f)
            return false;
        float left = (vx - wz * track_ * .5f) / radius_, right = (vx + wz * track_ * .5f) / radius_;
        float scale = std::max(1.0f, std::max(std::fabs(left), std::fabs(right)) / max_speed);
        float target[4] = {left / scale, left / scale, right / scale, right / scale};
        return wheels(target, ttl, seq, now);
    }
    void Motion::update(uint32_t now)
    {
        uint32_t elapsed = now - last_update_;
        last_update_ = now;
        data_.timestamp = now;
        data_.valid = 0;
        data_.command_age = now - command_time_;
        for (unsigned i = 0; i < 4; ++i)
        {
            data_.feedback[i] = motors_[i]->feedback();
            auto &f = data_.feedback[i];
            if (f.valid && now - f.timestamp_ms <= 20)
                data_.valid |= 1U << i;
            if (signs_[i] < 0 && (f.total_counts == std::numeric_limits<int64_t>::min() || f.delta_counts == std::numeric_limits<int32_t>::min()))
            {
                data_.valid &= ~(1U << i);
                f.valid = false;
                f.total_counts = 0;
                f.delta_counts = 0;
            }
            f.total_counts *= signs_[i];
            f.delta_counts *= signs_[i];
            f.rpm *= signs_[i];
            f.counts_per_second *= signs_[i];
        }
        if (data_.state != MotionState::Enabled)
            return;
        if (data_.valid != 15 || elapsed == 0 || elapsed > 20)
        {
            data_.fault = 2;
            stop(MotionState::Fault);
            return;
        }
        if (now - command_time_ >= ttl_)
        {
            stop(MotionState::Timeout);
            return;
        }
        float dt = elapsed * .001f;
        for (unsigned i = 0; i < 4; ++i)
        {
            float raw_measured = data_.feedback[i].rpm * .10471975512f;
            filtered_[i] += dt / (tuning[i].filter_seconds + dt) * (raw_measured - filtered_[i]);
            float measured = filtered_[i];
            float desired = data_.target[i];
            // Ramp to zero and wait for coast-down before applying reverse polarity.
            if (data_.ramp[i] * desired < 0 || (std::fabs(data_.ramp[i]) < .001f && raw_measured * desired < 0 && std::fabs(raw_measured) > .5f))
                desired = 0;
            float step = acceleration * dt;
            data_.ramp[i] += std::clamp(desired - data_.ramp[i], -step, step);
            if (std::fabs(data_.ramp[i]) < .001f)
            {
                integral_[i] = 0;
                starting_ms_[i] = moving_ms_[i] = stalled_ms_[i] = 0;
                started_[i] = false;
                data_.duty[i] = 0;
                motors_[i]->coast();
                continue;
            }
            const auto &g = tuning[i];
            float reference = data_.ramp[i];
            float direction = reference > 0 ? 1.0f : -1.0f;
            float error = reference - measured;
            if (started_[i] && std::fabs(desired) >= 1.0f && desired * reference > 0)
            {
                stalled_ms_[i] = raw_measured * direction < .25f ? stalled_ms_[i] + static_cast<uint16_t>(elapsed) : 0;
                if (stalled_ms_[i] >= 30U)
                {
                    if (data_.recoveries[i] >= 3U)
                    {
                        data_.fault = 5; // Repeated low-speed loss, do not hammer indefinitely.
                        stop(MotionState::Fault);
                        return;
                    }
                    ++data_.recoveries[i];
                    started_[i] = false;
                    starting_ms_[i] = moving_ms_[i] = stalled_ms_[i] = 0;
                    integral_[i] = 0;
                }
            }
            if (!started_[i])
            {
                float threshold = std::clamp(std::fabs(reference) * .5f, .5f, 1.0f);
                moving_ms_[i] = raw_measured * direction > threshold ? moving_ms_[i] + static_cast<uint16_t>(elapsed) : 0;
                if (moving_ms_[i] >= 20U)
                    started_[i] = true;
            }
            bool starting = !started_[i] && desired * reference > 0;
            if (starting)
            {
                starting_ms_[i] += static_cast<uint16_t>(elapsed);
                if (starting_ms_[i] > 300U)
                {
                    data_.fault = 4; // No encoder motion within bounded breakaway.
                    stop(MotionState::Fault);
                    return;
                }
            }
            // Fade compensation near zero rather than adding a fixed jump to
            // arbitrarily small commands. Start boost never renews from packets.
            float fade = std::min(std::fabs(reference), 1.0f);
            float feed = direction * g.friction * fade + g.velocity * reference;
            float candidate = std::clamp(integral_[i] + g.ki * error * dt, -100.0f, 100.0f);
            auto limit = [&](float raw)
            {
                float directed = reference > 0 ? std::clamp(raw, 0.0f, max_duty) : std::clamp(raw, -max_duty, 0.0f);
                if (starting)
                {
                    float extra = starting_ms_[i] > 100U ? (starting_ms_[i] - 100U) * .5f : 0.0f;
                    float boost = std::min(max_duty, g.breakaway + extra) * std::min(std::fabs(desired), 1.0f);
                    directed = direction * std::max(direction * directed, boost);
                }
                float step_duty = 1000.0f * dt;
                // Slew only during breakaway. Limiting running corrections causes
                // a delayed torque response and repeated low-speed loss.
                return starting ? direction * std::min(std::fabs(directed), std::fabs(float(data_.duty[i])) + step_duty) : directed;
            };
            float raw = feed + g.kp * error + candidate;
            float output = limit(raw);
            // Freeze during breakaway; conditional integration accounts for
            // amplitude, direction and slew limits, not just the PWM ceiling.
            if (!starting && (raw - output) * error <= 0)
                integral_[i] = candidate;
            output = limit(feed + g.kp * error + integral_[i]);
            data_.duty[i] = static_cast<int32_t>(std::lround(output));
            if (!motors_[i]->setDuty(data_.duty[i] * signs_[i]))
            {
                data_.fault = 3;
                stop(MotionState::Fault);
                return;
            }
        }
    }
}
