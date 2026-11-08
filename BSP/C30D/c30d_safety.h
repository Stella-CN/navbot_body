#ifndef C30D_SAFETY_H
#define C30D_SAFETY_H

#ifdef __cplusplus
extern "C"
{
#endif

    /* Callable during initialization and faults; no HAL handles or RTOS needed. */
    void C30D_ForceMotorIdle(void);
#ifdef __cplusplus
    [[noreturn]] void C30D_TemplatePanic(void);
#else
_Noreturn void C30D_TemplatePanic(void);
#endif

#ifdef __cplusplus
}
#endif
#endif
