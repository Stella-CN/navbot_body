#ifndef C30D_SAFETY_H
#define C30D_SAFETY_H

/* Callable during initialization and faults; no HAL handles or RTOS needed. */
void C30D_ForceMotorIdle(void);
_Noreturn void C30D_TemplatePanic(void);

#endif
