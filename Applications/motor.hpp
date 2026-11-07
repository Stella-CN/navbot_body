#ifndef NAVBOT_MOTOR_HPP
#define NAVBOT_MOTOR_HPP
#include <cstdint>

namespace navbot
{
    enum class WheelPosition
    {
        FrontLeft,
        FrontRight,
        RearLeft,
        RearRight
    };
    enum class MotorState
    {
        Uninitialized,
        Disabled,
        Enabled,
        Driving,
        Braking
    };
    enum class MotorError
    {
        None,
        NotReady,
        InvalidArgument,
        Hardware,
        SampleInterval,
        CounterRange
    };

    struct MotorInfo
    {
        WheelPosition wheel;
        char connector;
        const char *model; // Static string, no ownership transfer.
        uint16_t rated_mv;
        uint16_t gear_ratio;
        uint16_t encoder_ppr;
        uint8_t quadrature;
        uint8_t counter_bits;
        bool counts_calibrated;
        constexpr uint32_t countsPerRevolution() const
        {
            return uint32_t(gear_ratio) * encoder_ppr * quadrature;
        }
    };

    struct MotorFeedback
    {
        int64_t total_counts{};
        int32_t delta_counts{};
        float counts_per_second{};
        float rpm{}; // Based on MotorInfo; provisional until counts_calibrated.
        uint32_t timestamp_ms{};
        uint32_t raw_count{};
        bool valid{};
    };

    /* One control-task owner. Returned data are values; no ISR/other-task access.
     * No hardware operations in constructors; no allocations/copy/move.
     * Higher-level PID calls updateFeedback(), then setDuty(); it lives elsewhere. */
    class Motor
    {
    public:
        Motor(const Motor &) = delete;
        Motor &operator=(const Motor &) = delete;
        Motor(Motor &&) = delete;
        Motor &operator=(Motor &&) = delete;

        bool initialize(uint32_t now_ms);
        bool enable();
        void disable();
        bool setDuty(int32_t permille);
        void coast();
        bool brake();
        bool updateFeedback(uint32_t now_ms);
        /* Only while disabled: +1/-1 are electrical/encoder conventions, not
         * an assertion of vehicle-forward calibration. Reinitialize afterward. */
        bool configure(uint16_t duty_limit, int8_t drive_polarity, int8_t encoder_polarity);
        MotorInfo info() const { return info_; }
        MotorFeedback feedback() const { return feedback_; }
        MotorState state() const { return state_; }
        MotorError error() const { return error_; }
        int32_t duty() const { return duty_; }
        uint16_t dutyLimit() const { return duty_limit_; }
        int8_t drivePolarity() const { return drive_polarity_; }
        int8_t encoderPolarity() const { return encoder_polarity_; }

    protected:
        explicit constexpr Motor(MotorInfo info, int8_t encoder_polarity = 1)
            : info_(info), encoder_polarity_(encoder_polarity) {}
        ~Motor() = default; // Static ownership; never delete through Motor*.
        virtual bool hardwareInit() = 0;
        virtual bool hardwareWrite(int32_t duty) = 0;
        virtual void hardwareCoast() = 0;
        virtual bool hardwareBrake() = 0;
        virtual uint32_t hardwareCounter() const = 0;

    private:
        bool fail(MotorError error);
        bool enabled() const;
        const MotorInfo info_;
        MotorFeedback feedback_{};
        MotorState state_{MotorState::Uninitialized};
        MotorError error_{MotorError::None};
        uint32_t previous_count_{};
        uint16_t duty_limit_{200};
        int8_t drive_polarity_{1}, encoder_polarity_{1};
        int32_t duty_{};
    };
}
#endif
