#ifndef NAVBOT_WHEEL_MOTORS_HPP
#define NAVBOT_WHEEL_MOTORS_HPP
#include "motor.hpp"
#include "c30d_motor.h"

namespace navbot
{
    class WheelMotors;
    class C30dMotor : public Motor
    {
    protected:
        constexpr C30dMotor(WheelPosition wheel, C30D_MotorChannel channel, uint8_t bits)
            : Motor({wheel, char('A' + channel), "MG513P10-12V GMR", 12000, 10, 500, 4, bits, false},
                    channel == C30D_MOTOR_A || channel == C30D_MOTOR_D ? -1 : 1),
              channel_(channel) {}
        ~C30dMotor() = default;

    private:
        bool hardwareInit() final;
        bool hardwareWrite(int32_t duty) final;
        void hardwareCoast() final;
        bool hardwareBrake() final;
        uint32_t hardwareCounter() const final;
        const C30D_MotorChannel channel_;
    };

    class FrontLeftMotor final : public C30dMotor
    {
        friend class WheelMotors;
        constexpr FrontLeftMotor() : C30dMotor(WheelPosition::FrontLeft, C30D_MOTOR_C, 16) {}
    };
    class FrontRightMotor final : public C30dMotor
    {
        friend class WheelMotors;
        constexpr FrontRightMotor() : C30dMotor(WheelPosition::FrontRight, C30D_MOTOR_B, 16) {}
    };
    class RearLeftMotor final : public C30dMotor
    {
        friend class WheelMotors;
        constexpr RearLeftMotor() : C30dMotor(WheelPosition::RearLeft, C30D_MOTOR_D, 32) {}
    };
    class RearRightMotor final : public C30dMotor
    {
        friend class WheelMotors;
        constexpr RearRightMotor() : C30dMotor(WheelPosition::RearRight, C30D_MOTOR_A, 32) {}
    };

    /* One fixed set of wheel objects; only controlTask accesses this API.
     * instance() performs no I/O. initialize() starts encoders, never PWM.
     * No blanket enable: each motor requires an explicit enable and command. */
    class WheelMotors final
    {
    public:
        static WheelMotors &instance();
        WheelMotors(const WheelMotors &) = delete;
        WheelMotors &operator=(const WheelMotors &) = delete;
        bool initialize(uint32_t now_ms);
        bool updateFeedback(uint32_t now_ms);
        void disableAll();
        FrontLeftMotor &frontLeft() { return front_left_; }
        FrontRightMotor &frontRight() { return front_right_; }
        RearLeftMotor &rearLeft() { return rear_left_; }
        RearRightMotor &rearRight() { return rear_right_; }

    private:
        constexpr WheelMotors() = default;
        FrontLeftMotor front_left_;
        FrontRightMotor front_right_;
        RearLeftMotor rear_left_;
        RearRightMotor rear_right_;
    };
}
#endif
