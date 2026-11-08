#include "imu_diagnostics.h"
#include "c30d_soft_i2c.h"
#include "c30d_oled.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

/* Only sensorTask publishes. monitorTask consumes by value. GDB reads halted. */
static C30D_ImuSnapshot navbot_imu_snapshot;
uint32_t navbot_oled_frames, navbot_oled_max_us;
uint32_t navbot_stack_free_words[4];
static Navbot_AttitudeSnapshot navbot_attitude_snapshot;
void Navbot_ImuPublish(const C30D_ImuSnapshot *snapshot, const Navbot_AttitudeSnapshot *attitude)
{
    taskENTER_CRITICAL();
    navbot_imu_snapshot = *snapshot;
    navbot_attitude_snapshot = *attitude;
    taskEXIT_CRITICAL();
}
void Navbot_ImuSnapshot(C30D_ImuSnapshot *snapshot, Navbot_AttitudeSnapshot *attitude)
{
    taskENTER_CRITICAL();
    *snapshot = navbot_imu_snapshot;
    *attitude = navbot_attitude_snapshot;
    taskEXIT_CRITICAL();
}
static void angle_line(unsigned row, const char *name, float degrees, int valid)
{
    char text[24];
    if (valid)
    {
        long centi = (long)(degrees * 100.0f);
        char sign = centi < 0 ? '-' : '+';
        if (centi < 0)
        {
            centi = -centi;
        }
        (void)snprintf(text, sizeof(text), "%s %c%3ld.%02ld DEG", name, sign, centi / 100, centi % 100);
    }
    else
    {
        (void)snprintf(text, sizeof(text), "%s   ---- DEG", name);
    }
    C30D_OledLine(row, text);
}
void Navbot_ImuDisplay(uint32_t now)
{
    C30D_ImuSnapshot d;
    Navbot_AttitudeSnapshot a;
    Navbot_ImuSnapshot(&d, &a);
    uint32_t age = now - d.timestamp_ms;
    int imu_valid = d.valid != 0U && age < 30U;
    int valid = imu_valid && a.valid != 0U && (uint32_t)(now - a.timestamp_ms) < 30U;
    char line[32];
    C30D_OledLine(0, valid ? "IMU OK  MOTOR OFF" : (imu_valid && !a.calibrated ? "IMU CAL MOTOR OFF" : "IMU BAD MOTOR OFF"));
    if (a.calibration_state == NAVBOT_ATTITUDE_WARMUP)
    {
        C30D_OledLine(1, "WARMUP KEEP STILL");
    }
    else if (!a.calibrated)
    {
        (void)snprintf(line, sizeof(line), "CAL %lu/500 KEEP STILL", (unsigned long)a.calibration_samples);
        C30D_OledLine(1, line);
    }
    else
    {
        C30D_OledLine(1, "6AXIS YAW REL DEG");
    }
    angle_line(2, "ROLL ", a.roll_deg, valid);
    angle_line(3, "PITCH", a.pitch_deg, valid);
    angle_line(4, "YAW  ", a.yaw_deg, valid);
    (void)snprintf(line, sizeof(line), "N%lu DT%luUS", (unsigned long)(d.sequence % 100000U), (unsigned long)d.period_us);
    C30D_OledLine(5, line);
    (void)snprintf(line, sizeof(line), "H%lu E%lu C%lu EP%lu", (unsigned long)d.health, (unsigned long)d.last_error,
                   (unsigned long)a.calibration_state, (unsigned long)(a.heading_epoch % 10000U));
    C30D_OledLine(6, line);
    (void)snprintf(line, sizeof(line), "AGE%luMS R%lu", (unsigned long)(age > 9999U ? 9999U : age), (unsigned long)(d.recovery_count % 1000000U));
    C30D_OledLine(7, line);
    uint32_t start = C30D_Cycles();
    C30D_OledRefresh();
    uint32_t elapsed = C30D_ElapsedUs(start);
    if (elapsed > navbot_oled_max_us)
    {
        navbot_oled_max_us = elapsed;
    }
    ++navbot_oled_frames;
}
