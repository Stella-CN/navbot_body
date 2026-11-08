#include "attitude.h"
#include <math.h>
#include <string.h>

#define GRAVITY 9.80665f
#define RAD_TO_DEG 57.2957795130823f
#define PROPORTIONAL_GAIN 2.0f
#define WARMUP_MS 2000U
#define STALE_MS 30U

static float clamp_unit(float x) { return fminf(1.0f, fmaxf(-1.0f, x)); }
static float norm3(const float v[3]) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
static void reset_window(Navbot_Attitude *a)
{
    if (a->data.calibration_samples != 0U)
    {
        ++a->data.calibration_resets;
    }
    a->data.calibration_samples = 0;
    memset(a->gyro_mean, 0, sizeof(a->gyro_mean));
    memset(a->gyro_m2, 0, sizeof(a->gyro_m2));
    memset(a->accel_mean, 0, sizeof(a->accel_mean));
    memset(a->accel_m2, 0, sizeof(a->accel_m2));
    a->temperature_mean = 0;
}
static void euler(Navbot_AttitudeSnapshot *d)
{
    const float *q = d->q;
    d->roll_deg = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * RAD_TO_DEG;
    d->pitch_deg = asinf(clamp_unit(2.0f * (q[0] * q[2] - q[3] * q[1]))) * RAD_TO_DEG;
    d->yaw_deg = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * RAD_TO_DEG;
}
static int initialize_tilt(Navbot_Attitude *a, const float accel[3], uint32_t timestamp)
{
    float n = norm3(accel);
    if (n < 0.85f * GRAVITY || n > 1.15f * GRAVITY)
    {
        return 0;
    }
    float roll = atan2f(accel[1], accel[2]);
    float pitch = atan2f(-accel[0], sqrtf(accel[1] * accel[1] + accel[2] * accel[2]));
    float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    a->data.q[0] = cr * cp;
    a->data.q[1] = sr * cp;
    a->data.q[2] = cr * sp;
    a->data.q[3] = -sr * sp;
    a->last_timestamp_ms = timestamp;
    a->have_previous = 1;
    ++a->data.heading_epoch;
    a->data.valid = 1;
    a->data.timestamp_ms = timestamp;
    euler(&a->data);
    return 1;
}
static void mean_update(float x, uint32_t count, float *mean, float *m2)
{
    float delta = x - *mean;
    *mean += delta / (float)count;
    *m2 += delta * (x - *mean);
}
static void calibrate(Navbot_Attitude *a, const C30D_ImuSnapshot *s)
{
    Navbot_AttitudeSnapshot *d = &a->data;
    float an = norm3(s->accel_m_s2);
    /* Limits reject visible motion/vibration; slow constant yaw cannot be
     * distinguished from gyro bias with six axes. User must keep board still. */
    int stationary = an >= 0.90f * GRAVITY && an <= 1.10f * GRAVITY && norm3(s->gyro_rad_s) < 0.08f;
    if (d->calibration_samples != 0U)
    {
        float delta[3];
        for (unsigned i = 0; i < 3U; ++i)
        {
            delta[i] = s->accel_m_s2[i] - a->accel_mean[i];
        }
        if (norm3(delta) > 0.15f)
        {
            stationary = 0;
        }
    }
    if (!stationary)
    {
        ++d->calibration_rejections;
        reset_window(a);
        return;
    }
    uint32_t n = ++d->calibration_samples;
    for (unsigned i = 0; i < 3U; ++i)
    {
        mean_update(s->gyro_rad_s[i], n, &a->gyro_mean[i], &a->gyro_m2[i]);
        mean_update(s->accel_m_s2[i], n, &a->accel_mean[i], &a->accel_m2[i]);
    }
    a->temperature_mean += (s->temperature_c - a->temperature_mean) / (float)n;
    if (n < NAVBOT_CALIBRATION_SAMPLES)
    {
        return;
    }
    int stable = 1;
    for (unsigned i = 0; i < 3U; ++i)
    {
        d->gyro_std_rad_s[i] = sqrtf(fmaxf(0, a->gyro_m2[i] / (float)(n - 1U)));
        d->accel_std_m_s2[i] = sqrtf(fmaxf(0, a->accel_m2[i] / (float)(n - 1U)));
        if (d->gyro_std_rad_s[i] > 0.003f || d->accel_std_m_s2[i] > 0.08f)
        {
            stable = 0;
        }
    }
    if (!stable)
    {
        ++d->calibration_rejections;
        reset_window(a);
        return;
    }
    memcpy(d->gyro_bias_rad_s, a->gyro_mean, sizeof(d->gyro_bias_rad_s));
    d->calibration_temperature_c = a->temperature_mean;
    ++d->calibration_runs;
    d->calibrated = 1;
    d->calibration_state = NAVBOT_ATTITUDE_READY;
    (void)initialize_tilt(a, a->accel_mean, s->timestamp_ms);
}
void Navbot_AttitudeInit(Navbot_Attitude *a)
{
    memset(a, 0, sizeof(*a));
    a->data.q[0] = 1.0f;
}
void Navbot_AttitudeUpdate(Navbot_Attitude *a, const C30D_ImuSnapshot *s, uint32_t now)
{
    Navbot_AttitudeSnapshot *d = &a->data;
    int finite = isfinite(s->temperature_c);
    for (unsigned i = 0; i < 3U; ++i)
    {
        finite = finite && isfinite(s->accel_m_s2[i]) && isfinite(s->gyro_rad_s[i]);
    }
    if (s->valid == 0U || (uint32_t)(now - s->timestamp_ms) >= STALE_MS || !finite)
    {
        if (a->have_previous != 0U)
        {
            ++d->input_errors;
        }
        if (!finite)
        {
            ++d->numeric_errors;
        }
        d->valid = 0;
        a->have_previous = 0;
        if (!d->calibrated)
        {
            reset_window(a);
            a->warmup_started = 0;
            d->calibration_state = NAVBOT_ATTITUDE_WARMUP;
        }
        return;
    }
    if (s->recovery_count != a->last_recovery_count)
    {
        /* Driver recovery resets the IMU. Bench testing found the previous bias
         * could shift afterwards; never publish it as a valid calibration. */
        a->last_recovery_count = s->recovery_count;
        d->valid = 0;
        d->calibrated = 0;
        d->calibration_state = NAVBOT_ATTITUDE_WARMUP;
        reset_window(a);
        a->warmup_started = 0;
        a->have_previous = 0;
        a->seen_sample = 0;
    }
    if (a->seen_sample && s->sequence == a->last_sequence)
    {
        return;
    }
    a->seen_sample = 1;
    a->last_sequence = s->sequence;
    d->accel_norm_g = norm3(s->accel_m_s2) / GRAVITY;
    if (!d->calibrated)
    {
        if (!a->warmup_started)
        {
            a->warmup_start_ms = s->timestamp_ms;
            a->warmup_started = 1;
        }
        if ((uint32_t)(s->timestamp_ms - a->warmup_start_ms) < WARMUP_MS)
        {
            return;
        }
        d->calibration_state = NAVBOT_ATTITUDE_CALIBRATING;
        /* Calibration must be consecutive: a 30ms gap discards the window. */
        if (d->calibration_samples != 0U && (uint32_t)(s->timestamp_ms - a->last_timestamp_ms) > 20U)
        {
            reset_window(a);
        }
        a->last_timestamp_ms = s->timestamp_ms;
        calibrate(a, s);
        return;
    }
    for (unsigned i = 0; i < 3U; ++i)
    {
        d->gyro_corrected_rad_s[i] = s->gyro_rad_s[i] - d->gyro_bias_rad_s[i];
    }
    if (!a->have_previous)
    {
        (void)initialize_tilt(a, s->accel_m_s2, s->timestamp_ms);
        return;
    }
    uint32_t elapsed_ms = s->timestamp_ms - a->last_timestamp_ms;
    a->last_timestamp_ms = s->timestamp_ms;
    if (elapsed_ms < 5U || elapsed_ms > 20U || s->period_us < 5000U || s->period_us > 20000U)
    {
        ++d->dt_rejects;
        d->valid = 0;
        a->have_previous = 0;
        return;
    }
    d->dt_s = (float)s->period_us * 0.000001f;
    float gx = d->gyro_corrected_rad_s[0], gy = d->gyro_corrected_rad_s[1], gz = d->gyro_corrected_rad_s[2];
    const float *q = d->q;
    /* Accelerometer correction is bounded to plausible gravity magnitude.
     * It cannot observe yaw or distinguish all sustained linear accelerations. */
    if (d->accel_norm_g >= 0.85f && d->accel_norm_g <= 1.15f)
    {
        float ax = s->accel_m_s2[0] / (d->accel_norm_g * GRAVITY);
        float ay = s->accel_m_s2[1] / (d->accel_norm_g * GRAVITY);
        float az = s->accel_m_s2[2] / (d->accel_norm_g * GRAVITY);
        float vx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
        float vy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
        float vz = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
        gx += PROPORTIONAL_GAIN * (ay * vz - az * vy);
        gy += PROPORTIONAL_GAIN * (az * vx - ax * vz);
        gz += PROPORTIONAL_GAIN * (ax * vy - ay * vx);
    }
    else
    {
        ++d->accel_rejects;
    }
    float half_dt = 0.5f * d->dt_s;
    float next[4] = {q[0] + (-q[1] * gx - q[2] * gy - q[3] * gz) * half_dt,
                     q[1] + (q[0] * gx + q[2] * gz - q[3] * gy) * half_dt,
                     q[2] + (q[0] * gy - q[1] * gz + q[3] * gx) * half_dt,
                     q[3] + (q[0] * gz + q[1] * gy - q[2] * gx) * half_dt};
    float n = sqrtf(next[0] * next[0] + next[1] * next[1] + next[2] * next[2] + next[3] * next[3]);
    if (!isfinite(n) || n < 0.5f || n > 1.5f)
    {
        ++d->numeric_errors;
        d->valid = 0;
        a->have_previous = 0;
        return;
    }
    for (unsigned i = 0; i < 4U; ++i)
    {
        d->q[i] = next[i] / n;
    }
    euler(d);
    ++d->updates;
    d->timestamp_ms = s->timestamp_ms;
    d->valid = 1;
}
