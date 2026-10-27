#include "app_tasks.h"
#include "chassis_link.h"
#include "cmsis_os2.h"
#include "c30d_safety.h"
#include "imu_diagnostics.h"
#include "c30d_soft_i2c.h"
#include "c30d_oled.h"
#include "FreeRTOS.h"
#include "task.h"
extern osThreadId_t controlTaskHandle, commTaskHandle, sensorTaskHandle, monitorTaskHandle;

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
    Navbot_LinkInit();
    deadline = osKernelGetTickCount();
    for (;;)
    {
        Navbot_LinkPoll(osKernelGetTickCount());
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
    static C30D_Imu imu;
    static Navbot_Attitude attitude;
    C30D_ImuInit(&imu);
    Navbot_AttitudeInit(&attitude);
    for (;;)
    {
        uint32_t start = C30D_Cycles();
        C30D_ImuPoll(&imu, osKernelGetTickCount());
        Navbot_AttitudeUpdate(&attitude, &imu.data, osKernelGetTickCount());
        uint32_t elapsed = C30D_ElapsedUs(start);
        if (elapsed > imu.data.execution_max_us)
        {
            imu.data.execution_max_us = elapsed;
        }
        Navbot_ImuPublish(&imu.data, &attitude.data);
        deadline += period;
        if (osDelayUntil(deadline) != osOK)
        {
            ++imu.data.deadline_misses;
            imu.data.valid = 0;
            Navbot_AttitudeUpdate(&attitude, &imu.data, osKernelGetTickCount());
            deadline = osKernelGetTickCount();
            Navbot_ImuPublish(&imu.data, &attitude.data);
            (void)osDelay(period);
            deadline = osKernelGetTickCount();
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
    C30D_OledInit();
    deadline = osKernelGetTickCount();
    for (;;)
    {
        Navbot_ImuDisplay(osKernelGetTickCount());
        const osThreadId_t handles[4] = {controlTaskHandle, commTaskHandle, sensorTaskHandle, monitorTaskHandle};
        for (unsigned i = 0; i < 4U; ++i)
        {
            navbot_stack_free_words[i] = uxTaskGetStackHighWaterMark((TaskHandle_t)handles[i]);
        }
        deadline += period;
        if (osDelayUntil(deadline) != osOK)
        {
            C30D_TemplatePanic();
        }
    }
}
