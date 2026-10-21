/* Deterministic tests of the actual portable calibration/fusion module. */
#include "attitude.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define G 9.80665f
#define RAD 0.0174532925199433f
static uint32_t tick;
static C30D_ImuSnapshot sample;
static const float bias[3] = {0.01f, -0.02f, 0.005f};
static void next(Navbot_Attitude *a, uint32_t ms)
{
    tick += ms;
    sample.timestamp_ms = tick;
    ++sample.sequence;
    sample.period_us = ms * 1000U;
    Navbot_AttitudeUpdate(a, &sample, tick);
}
static void reset(Navbot_Attitude *a, float roll, float pitch)
{
    Navbot_AttitudeInit(a);
    sample = (C30D_ImuSnapshot){0};
    tick = 0;
    sample.valid = 1;
    sample.temperature_c = 30;
    sample.accel_m_s2[0] = -G * sinf(pitch);
    sample.accel_m_s2[1] = G * sinf(roll) * cosf(pitch);
    sample.accel_m_s2[2] = G * cosf(roll) * cosf(pitch);
    for (unsigned i = 0; i < 3; ++i)
    {
        sample.gyro_rad_s[i] = bias[i];
    }
}
static void calibrate(Navbot_Attitude *a, float roll, float pitch)
{
    reset(a, roll, pitch);
    for (unsigned i = 0; i < 705; ++i)
    {
        next(a, 10);
    }
    assert(a->data.valid && a->data.calibrated && a->data.calibration_samples == 500);
    for (unsigned i = 0; i < 3; ++i)
    {
        assert(fabsf(a->data.gyro_bias_rad_s[i] - bias[i]) < 1e-6f);
    }
    assert(fabsf(a->data.roll_deg - roll / RAD) < 0.01f);
    assert(fabsf(a->data.pitch_deg - pitch / RAD) < 0.01f);
    assert(fabsf(a->data.yaw_deg) < 0.01f);
}
static void check_norm(const Navbot_Attitude *a)
{
    float n = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
        assert(isfinite(a->data.q[i]));
        n += a->data.q[i] * a->data.q[i];
    }
    assert(fabsf(n - 1) < 1e-5f);
}
int main(void)
{
    Navbot_Attitude a;
    calibrate(&a, 0.2f, -0.3f);
    for (unsigned i = 0; i < 10000; ++i)
    {
        next(&a, 10);
    }
    assert(fabsf(a.data.roll_deg - 0.2f / RAD) < 0.01f && fabsf(a.data.yaw_deg) < 0.01f);
    check_norm(&a);
    /* World-Z turn while tilted: body angular rate is gravity direction * rate. */
    float r = 0.2f, p = -0.3f;
    for (unsigned i = 0; i < 1000; ++i)
    {
        for (unsigned j = 0; j < 3; ++j)
        {
            sample.gyro_rad_s[j] = bias[j] + sample.accel_m_s2[j] / G * (9 * RAD);
        }
        next(&a, (i % 2) ? 15 : 5);
    }
    assert(fabsf(a.data.yaw_deg - 90) < 0.03f);
    assert(fabsf(a.data.roll_deg - r / RAD) < 0.03f && fabsf(a.data.pitch_deg - p / RAD) < 0.03f);
    calibrate(&a, 0, 0);
    sample.gyro_rad_s[2] = bias[2] + 0.001f;
    for (unsigned i = 0; i < 10000; ++i)
    {
        next(&a, 10);
    }
    assert(fabsf(a.data.yaw_deg - 0.1f / RAD) < 0.03f); /* no yaw deadband/lock */
    calibrate(&a, 0, 0);
    for (unsigned i = 1; i <= 300; ++i)
    {
        float angle = (float)i * 0.1f * RAD;
        sample.accel_m_s2[1] = G * sinf(angle);
        sample.accel_m_s2[2] = G * cosf(angle);
        sample.gyro_rad_s[0] = bias[0] + 10 * RAD;
        next(&a, 10);
    }
    assert(fabsf(a.data.roll_deg - 30) < 0.2f);
    check_norm(&a);
    calibrate(&a, 0, 0);
    for (unsigned i = 1; i <= 300; ++i)
    {
        float angle = (float)i * 0.1f * RAD;
        sample.accel_m_s2[0] = -G * sinf(angle);
        sample.accel_m_s2[2] = G * cosf(angle);
        sample.gyro_rad_s[1] = bias[1] + 10 * RAD;
        next(&a, 10);
    }
    assert(fabsf(a.data.pitch_deg - 30) < 0.2f);
    check_norm(&a);
    /* Norm gate rejects translational acceleration, not the gyro integration. */
    uint32_t rejected = a.data.accel_rejects;
    sample.accel_m_s2[2] = 2 * G;
    next(&a, 10);
    assert(a.data.accel_rejects == rejected + 1);
    uint32_t epoch = a.data.heading_epoch;
    sample.valid = 0;
    next(&a, 10);
    assert(!a.data.valid);
    sample.valid = 1;
    sample.accel_m_s2[0] = 0;
    sample.accel_m_s2[1] = 0;
    sample.accel_m_s2[2] = G;
    for (unsigned i = 0; i < 3; ++i)
    {
        sample.gyro_rad_s[i] = bias[i];
    }
    next(&a, 10);
    assert(a.data.valid && a.data.heading_epoch == epoch + 1 && fabsf(a.data.yaw_deg) < 0.01f);
    next(&a, 100);
    assert(!a.data.valid && a.data.dt_rejects > 0);
    next(&a, 10);
    assert(a.data.valid);
    sample.accel_m_s2[0] = NAN;
    next(&a, 10);
    assert(!a.data.valid && a.data.numeric_errors > 0);
    sample.accel_m_s2[0] = 0;
    next(&a, 10);
    assert(a.data.valid);
    check_norm(&a);
    /* Slow variable vibration must reject a calibration window on variance. */
    reset(&a, 0, 0);
    for (unsigned i = 0; i < 200; ++i)
    {
        next(&a, 10);
    }
    for (unsigned i = 0; i < 500; ++i)
    {
        sample.gyro_rad_s[0] = bias[0] + ((i % 2) ? 0.01f : -0.01f);
        next(&a, 10);
    }
    assert(!a.data.calibrated && a.data.calibration_rejections > 0);
    sample.gyro_rad_s[0] = bias[0];
    for (unsigned i = 0; i < 505; ++i)
    {
        next(&a, 10);
    }
    assert(a.data.calibrated);
    reset(&a, 0, 0);
    for (unsigned i = 0; i < 300; ++i)
    {
        next(&a, 10);
    }
    assert(a.data.calibration_samples > 0);
    sample.gyro_rad_s[2] = 0.2f;
    next(&a, 10);
    assert(!a.data.calibrated && a.data.calibration_samples == 0);
    sample.gyro_rad_s[2] = bias[2];
    for (unsigned i = 0; i < 100; ++i)
    {
        next(&a, 10);
    }
    sample.valid = 0;
    next(&a, 10);
    assert(a.data.calibration_samples == 0 && a.data.calibration_state == NAVBOT_ATTITUDE_WARMUP);
    /* Stale and duplicate values cannot advance calibration or attitude. */
    calibrate(&a, 0, 0);
    uint32_t updates = a.data.updates;
    Navbot_AttitudeUpdate(&a, &sample, tick);
    assert(a.data.updates == updates);
    Navbot_AttitudeUpdate(&a, &sample, tick + 30);
    assert(!a.data.valid);
    sample.timestamp_ms = 0xfffffff0U;
    tick = sample.timestamp_ms;
    next(&a, 10);
    next(&a, 10);
    next(&a, 10);
    assert(a.data.valid);
    check_norm(&a);
    /* A reset/recovery of the physical IMU invalidates its old zero-bias
     * calibration. A changed static bias must be relearned before valid output. */
    calibrate(&a, 0, 0);
    sample.recovery_count = 1;
    sample.gyro_rad_s[2] = bias[2] + 0.002f;
    next(&a, 10);
    assert(!a.data.calibrated && !a.data.valid && a.data.calibration_state == NAVBOT_ATTITUDE_WARMUP);
    for (unsigned i = 0; i < 705; ++i)
    {
        next(&a, 10);
    }
    assert(a.data.valid && a.data.calibrated && a.data.calibration_runs == 2);
    assert(a.data.heading_epoch == 2 && fabsf(a.data.gyro_bias_rad_s[2] - bias[2] - 0.002f) < 1e-6f);
    /* Crossing +/-180 must remain finite and wrap, not clamp or grow forever. */
    calibrate(&a, 0, 0);
    sample.gyro_rad_s[2] = bias[2] + 90 * RAD;
    for (unsigned i = 0; i < 300; ++i)
    {
        next(&a, 10);
    }
    assert(fabsf(a.data.yaw_deg + 90) < 0.1f);
    check_norm(&a);
    puts("PASS: gyro bias, tilted calibration, stationary stability, roll/pitch/yaw rotations, variable dt, slow yaw, motion/variance rejection, faults/NaN/gaps, tick/angle wrap");
}
