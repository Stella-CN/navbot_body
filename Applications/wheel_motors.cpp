#include "wheel_motors.hpp"
#include <initializer_list>

namespace navbot
{
    bool C30dMotor::hardwareInit() { return C30D_MotorInit(channel_) != 0; }
    bool C30dMotor::hardwareWrite(int32_t duty) { return C30D_MotorWrite(channel_, duty) != 0; }
    void C30dMotor::hardwareCoast() { C30D_MotorCoast(channel_); }
    bool C30dMotor::hardwareBrake() { return C30D_MotorBrake(channel_) != 0; }
    uint32_t C30dMotor::hardwareCounter() const { return C30D_MotorCounter(channel_); }

    WheelMotors &WheelMotors::instance()
    {
        static WheelMotors wheels;
        return wheels;
    }

    bool WheelMotors::initialize(uint32_t now)
    {
        disableAll();
        bool ok = true;
        for (Motor *motor : {static_cast<Motor *>(&front_left_), static_cast<Motor *>(&front_right_),
                             static_cast<Motor *>(&rear_left_), static_cast<Motor *>(&rear_right_)})
        {
            if (!motor->initialize(now))
            {
                ok = false;
            }
        }
        if (!ok)
        {
            disableAll();
        }
        return ok;
    }

    bool WheelMotors::updateFeedback(uint32_t now)
    {
        bool ok = true;
        for (Motor *motor : {static_cast<Motor *>(&front_left_), static_cast<Motor *>(&front_right_),
                             static_cast<Motor *>(&rear_left_), static_cast<Motor *>(&rear_right_)})
        {
            if (!motor->updateFeedback(now))
            {
                ok = false;
            }
        }
        if (!ok)
        {
            disableAll();
        }
        return ok;
    }

    void WheelMotors::disableAll()
    {
        front_left_.disable();
        front_right_.disable();
        rear_left_.disable();
        rear_right_.disable();
    }
}
