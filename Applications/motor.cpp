#include "motor.hpp"
#include <limits>

namespace navbot
{
    bool Motor::enabled() const
    {
        return state_ == MotorState::Enabled || state_ == MotorState::Driving || state_ == MotorState::Braking;
    }

    bool Motor::fail(MotorError error)
    {
        disable();
        error_ = error;
        return false;
    }

    bool Motor::initialize(uint32_t now)
    {
        disable();
        state_ = MotorState::Uninitialized;
        if ((info_.counter_bits != 16U && info_.counter_bits != 32U) || info_.gear_ratio == 0U ||
            info_.encoder_ppr == 0U || info_.quadrature != 4U ||
            uint64_t(info_.gear_ratio) * info_.encoder_ppr * info_.quadrature > UINT32_MAX)
        {
            error_ = MotorError::InvalidArgument;
            return false;
        }
        if (!hardwareInit())
        {
            error_ = MotorError::Hardware;
            return false;
        }
        previous_count_ = hardwareCounter();
        feedback_ = {};
        feedback_.raw_count = previous_count_;
        feedback_.timestamp_ms = now;
        state_ = MotorState::Disabled;
        error_ = MotorError::None;
        return true;
    }

    bool Motor::enable()
    {
        if (state_ != MotorState::Disabled || error_ != MotorError::None || !feedback_.valid)
        {
            return false;
        }
        hardwareCoast();
        duty_ = 0;
        state_ = MotorState::Enabled;
        return true;
    }

    void Motor::disable()
    {
        hardwareCoast();
        duty_ = 0;
        if (state_ != MotorState::Uninitialized)
        {
            state_ = MotorState::Disabled;
        }
    }

    bool Motor::setDuty(int32_t permille)
    {
        if (!enabled())
        {
            return fail(MotorError::NotReady);
        }
        if (permille < -int32_t(duty_limit_) || permille > int32_t(duty_limit_))
        {
            return fail(MotorError::InvalidArgument);
        }
        if (!hardwareWrite(permille * drive_polarity_))
        {
            return fail(MotorError::Hardware);
        }
        duty_ = permille;
        state_ = permille == 0 ? MotorState::Enabled : MotorState::Driving;
        return true;
    }

    void Motor::coast()
    {
        hardwareCoast();
        duty_ = 0;
        if (enabled())
        {
            state_ = MotorState::Enabled;
        }
    }

    bool Motor::brake()
    {
        if (!enabled())
        {
            return fail(MotorError::NotReady);
        }
        if (!hardwareBrake())
        {
            return fail(MotorError::Hardware);
        }
        duty_ = 0;
        state_ = MotorState::Braking;
        return true;
    }

    bool Motor::configure(uint16_t limit, int8_t drive, int8_t encoder)
    {
        if (enabled())
        {
            return false;
        }
        if (limit == 0U || limit > 1000U || (drive != 1 && drive != -1) || (encoder != 1 && encoder != -1))
        {
            return false;
        }
        duty_limit_ = limit;
        drive_polarity_ = drive;
        encoder_polarity_ = encoder;
        state_ = MotorState::Uninitialized;
        feedback_.valid = false;
        return true;
    }

    bool Motor::updateFeedback(uint32_t now)
    {
        if (state_ == MotorState::Uninitialized)
        {
            return false;
        }
        uint32_t current = hardwareCounter();
        uint32_t elapsed = now - feedback_.timestamp_ms;
        uint32_t mask = info_.counter_bits == 16U ? 0xffffU : 0xffffffffU;
        uint32_t raw_delta = (current - previous_count_) & mask;
        previous_count_ = current;
        feedback_.raw_count = current & mask;
        feedback_.timestamp_ms = now;
        feedback_.valid = false;
        feedback_.delta_counts = 0;
        feedback_.counts_per_second = 0.0f;
        feedback_.rpm = 0.0f;
        /* 500 Hz caller, <=20ms gaps. At the specified 1100 +/-132 rpm and
         * 20000 counts/rev, 16-bit delta cannot cross the half-range in 20ms.
         * Exact half-range is ambiguous; gaps must not silently fabricate speed. */
        if (elapsed == 0U || elapsed > 20U)
        {
            return fail(MotorError::SampleInterval);
        }
        if (raw_delta == (mask >> 1U) + 1U)
        {
            return fail(MotorError::CounterRange);
        }
        int64_t delta = raw_delta;
        if (raw_delta > (mask >> 1U))
        {
            delta -= int64_t(mask) + 1;
        }
        delta *= encoder_polarity_;
        if (delta > std::numeric_limits<int32_t>::max() || delta < std::numeric_limits<int32_t>::min() ||
            (delta > 0 && feedback_.total_counts > std::numeric_limits<int64_t>::max() - delta) ||
            (delta < 0 && feedback_.total_counts < std::numeric_limits<int64_t>::min() - delta))
        {
            return fail(MotorError::CounterRange);
        }
        feedback_.delta_counts = int32_t(delta);
        feedback_.total_counts += delta;
        feedback_.counts_per_second = float(delta) * 1000.0f / float(elapsed);
        feedback_.rpm = feedback_.counts_per_second * 60.0f / float(info_.countsPerRevolution());
        feedback_.valid = true;
        return true;
    }
}
