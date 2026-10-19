#include "app_tasks.h"
#include "cmsis_os2.h"
#include "c30d_safety.h"

void Navbot_ControlTask(void *argument)
{
    (void)argument;
    const uint32_t period = osKernelGetTickFreq() / 100U;
    uint32_t deadline = osKernelGetTickCount();
    if ((period == 0U) || ((osKernelGetTickFreq() % 100U) != 0U))
    {
        C30D_TemplatePanic();
    }
    /* Sole future owner of encoder deltas, wheel mapping, PI and motion state.
     * Remains DISABLED: no calibrated parameters or frozen protocol yet. */
    for (;;)
    {
        C30D_ForceMotorIdle();
        deadline += period;
        if (osDelayUntil(deadline) != osOK)
        {
            /* Missed deadline: latch the template rather than spin/catch up. */
            C30D_TemplatePanic();
        }
    }
}

void Navbot_CommTask(void *argument)
{
    (void)argument;
    const uint32_t period = osKernelGetTickFreq() / 200U;
    uint32_t deadline = osKernelGetTickCount();
    if ((period == 0U) || ((osKernelGetTickFreq() % 200U) != 0U))
    {
        C30D_TemplatePanic();
    }
    /* Future sole owner of USART3 RX parsing and TX buffers.
     * DMA is configured only; no reception, transmission or command acceptance. */
    for (;;)
    {

        deadline += period;
        if (osDelayUntil(deadline) != osOK)
        {
            /* Missed deadline: latch the template rather than spin/catch up. */
            C30D_TemplatePanic();
        }
    }
}

void Navbot_SensorTask(void *argument)
{
    (void)argument;
    const uint32_t period = osKernelGetTickFreq() / 100U;
    uint32_t deadline = osKernelGetTickCount();
    if ((period == 0U) || ((osKernelGetTickFreq() % 100U) != 0U))
    {
        C30D_TemplatePanic();
    }
    /* Future owner of bounded ADC/IMU acquisition and validity flags.
     * IMU pins remain inputs; no sensor driver or fabricated measurements. */
    for (;;)
    {

        deadline += period;
        if (osDelayUntil(deadline) != osOK)
        {
            /* Missed deadline: latch the template rather than spin/catch up. */
            C30D_TemplatePanic();
        }
    }
}

void Navbot_MonitorTask(void *argument)
{
    (void)argument;
    const uint32_t period = osKernelGetTickFreq() / 10U;
    uint32_t deadline = osKernelGetTickCount();
    if ((period == 0U) || ((osKernelGetTickFreq() % 10U) != 0U))
    {
        C30D_TemplatePanic();
    }
    /* Future owner of health aggregation and stack-watermark diagnostics.
     * Watchdog feeding must require fresh control and communication health. */
    for (;;)
    {

        deadline += period;
        if (osDelayUntil(deadline) != osOK)
        {
            /* Missed deadline: latch the template rather than spin/catch up. */
            C30D_TemplatePanic();
        }
    }
}
