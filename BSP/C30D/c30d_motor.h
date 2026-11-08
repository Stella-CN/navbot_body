#ifndef C30D_MOTOR_H
#define C30D_MOTOR_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        C30D_MOTOR_A,
        C30D_MOTOR_B,
        C30D_MOTOR_C,
        C30D_MOTOR_D
    } C30D_MotorChannel;
    /* Call only after CubeMX timer/GPIO initialization, from the control-task owner.
     * Init starts the free-running encoder and leaves this motor's output disabled. */
    int C30D_MotorInit(C30D_MotorChannel channel);
    /* Signed electrical duty, [-1000,1000] per mille. Zero disables both outputs.
     * Positive: first physical timer channel high, second PWM (slow decay).
     * This is not a vehicle-forward convention. Invalid commands stop this channel. */
    int C30D_MotorWrite(C30D_MotorChannel channel, int32_t duty_permille);
    void C30D_MotorCoast(C30D_MotorChannel channel);
    int C30D_MotorBrake(C30D_MotorChannel channel);
    uint32_t C30D_MotorCounter(C30D_MotorChannel channel);

#ifdef __cplusplus
}
#endif
#endif
