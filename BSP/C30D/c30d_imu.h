#ifndef C30D_IMU_H
#define C30D_IMU_H
#include <stdint.h>

typedef enum
{
    C30D_IMU_STARTING,
    C30D_IMU_OK,
    C30D_IMU_BUS_ERROR,
    C30D_IMU_ID_ERROR,
    C30D_IMU_CONFIG_ERROR,
    C30D_IMU_STALE
} C30D_ImuHealth;
typedef struct
{
    uint32_t sequence, timestamp_ms, valid, health, last_error, who_am_i;
    int16_t accel_raw[3], gyro_raw[3], temperature_raw;
    float accel_m_s2[3], gyro_rad_s[3], temperature_c;
    uint32_t period_us, period_min_us, period_max_us;
    uint32_t poll_period_us, poll_min_us, poll_max_us, execution_max_us;
    uint32_t nack_count, timeout_count, stuck_count, id_errors, config_errors;
    uint32_t not_ready_count, stale_count, retry_count, recovery_count, deadline_misses;
} C30D_ImuSnapshot;
typedef struct
{
    C30D_ImuSnapshot data;
    uint32_t state, due_ms, config_index, last_sample_cycles, last_poll_cycles;
    uint32_t ready_since_ms, had_error;
} C30D_Imu;
/* All driver state is owned by sensorTask; no internal allocation or blocking sleep. */
void C30D_ImuInit(C30D_Imu *imu);
void C30D_ImuPoll(C30D_Imu *imu, uint32_t now_ms);
#endif
