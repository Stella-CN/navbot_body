#ifndef NAVBOT_ATTITUDE_H
#define NAVBOT_ATTITUDE_H
#include "c30d_imu.h"

#define NAVBOT_CALIBRATION_SAMPLES 500U
/* Quaternion maps chip/body coordinates to a world frame whose +Z is up.
 * Euler convention: intrinsic ZYX, right-hand positive angles; yaw is relative.
 * A single stationary pose identifies gyro bias, not 3-axis accelerometer bias. */
typedef enum
{
    NAVBOT_ATTITUDE_WARMUP,
    NAVBOT_ATTITUDE_CALIBRATING,
    NAVBOT_ATTITUDE_READY
} Navbot_CalibrationState;
typedef struct
{
    uint32_t valid, calibrated, calibration_state, calibration_samples;
    uint32_t calibration_resets, calibration_rejections, calibration_runs, updates, heading_epoch;
    uint32_t timestamp_ms, input_errors, dt_rejects, accel_rejects, numeric_errors;
    float gyro_bias_rad_s[3], gyro_std_rad_s[3], accel_std_m_s2[3];
    float calibration_temperature_c, q[4], roll_deg, pitch_deg, yaw_deg;
    float gyro_corrected_rad_s[3], dt_s, accel_norm_g;
} Navbot_AttitudeSnapshot;
typedef struct
{
    Navbot_AttitudeSnapshot data;
    float gyro_mean[3], gyro_m2[3], accel_mean[3], accel_m2[3], temperature_mean;
    uint32_t warmup_start_ms, warmup_started, last_sequence, last_timestamp_ms;
    uint32_t have_previous, seen_sample, last_recovery_count;
} Navbot_Attitude;
/* Single sensor-task owner; no allocation, global filter state, or I/O. */
void Navbot_AttitudeInit(Navbot_Attitude *attitude);
void Navbot_AttitudeUpdate(Navbot_Attitude *attitude, const C30D_ImuSnapshot *sample, uint32_t now_ms);
#endif
