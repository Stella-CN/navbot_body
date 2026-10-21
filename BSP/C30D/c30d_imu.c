#include "c30d_imu.h"
#include "c30d_soft_i2c.h"
#include <string.h>
#include <limits.h>

#define ADDRESS 0x68U
#define STALE_MS 30U
/* 225Hz sensor ODR; application reads the latest complete burst at 100Hz.
 * +/-2g, +/-250dps, DLPF cfg3 (accel 50.4Hz, gyro 51.2Hz). */
static const struct
{
    uint8_t bank, reg, value;
} config[] = {
    {0, 0x06, 0x01}, {0, 0x07, 0x00}, {0, 0x05, 0x00}, {2, 0x00, 4}, {2, 0x01, 0x19}, {2, 0x10, 0}, {2, 0x11, 4}, {2, 0x14, 0x19}, {0, 0x11, 0x01}};
static int expired(uint32_t now, uint32_t due) { return (int32_t)(now - due) >= 0; }
static void failed(C30D_Imu *imu, uint32_t now, C30D_ImuHealth health, C30D_I2cStatus status)
{
    C30D_ImuSnapshot *d = &imu->data;
    d->valid = 0;
    d->health = health;
    d->last_error = health;
    if (status == C30D_I2C_NACK)
    {
        ++d->nack_count;
    }
    if (status == C30D_I2C_TIMEOUT)
    {
        ++d->timeout_count;
    }
    if (status == C30D_I2C_STUCK)
    {
        ++d->stuck_count;
    }
    imu->had_error = 1;
    imu->state = 0;
    imu->due_ms = now + 1000U;
    imu->last_sample_cycles = 0;
}
static int bus_ok(C30D_Imu *imu, uint32_t now, C30D_I2cStatus result)
{
    if (result == C30D_I2C_OK)
    {
        return 1;
    }
    failed(imu, now, C30D_IMU_BUS_ERROR, result);
    return 0;
}
static int16_t signed_word(const uint8_t *p)
{
    uint32_t u = ((uint32_t)p[0] << 8) | p[1];
    return (int16_t)(u >= 32768U ? (int32_t)u - 65536 : (int32_t)u);
}
static void range(uint32_t value, uint32_t *min, uint32_t *max)
{
    if (value < *min)
    {
        *min = value;
    }
    if (value > *max)
    {
        *max = value;
    }
}
void C30D_ImuInit(C30D_Imu *imu)
{
    memset(imu, 0, sizeof(*imu));
    imu->data.period_min_us = UINT32_MAX;
    imu->data.poll_min_us = UINT32_MAX;
    C30D_I2cInit();
}
void C30D_ImuPoll(C30D_Imu *imu, uint32_t now)
{
    C30D_ImuSnapshot *d = &imu->data;
    uint32_t cycles = C30D_Cycles();
    if (imu->last_poll_cycles != 0U)
    {
        d->poll_period_us = C30D_ElapsedUs(imu->last_poll_cycles);
        range(d->poll_period_us, &d->poll_min_us, &d->poll_max_us);
    }
    imu->last_poll_cycles = cycles;
    if (imu->state != 3U && d->retry_count != 0U && !expired(now, imu->due_ms))
    {
        return;
    }
    if (imu->state == 0U)
    {
        ++d->retry_count;
        if (!bus_ok(imu, now, C30D_I2cRecover()))
        {
            return;
        }
        if (!bus_ok(imu, now, C30D_I2cWrite(ADDRESS, 0x7f, 0)))
        {
            return;
        }
        if (!bus_ok(imu, now, C30D_I2cWrite(ADDRESS, 0x06, 0x80)))
        {
            return;
        }
        imu->state = 1;
        imu->due_ms = now + 100U;
        return;
    }
    if (imu->state == 1U)
    {
        uint8_t id = 0;
        if (!bus_ok(imu, now, C30D_I2cRead(ADDRESS, 0, &id, 1)))
        {
            return;
        }
        d->who_am_i = id;
        if (id != 0xeaU)
        {
            ++d->id_errors;
            failed(imu, now, C30D_IMU_ID_ERROR, C30D_I2C_OK);
            return;
        }
        if (!bus_ok(imu, now, C30D_I2cWrite(ADDRESS, 6, 1)))
        {
            return;
        }
        imu->state = 2;
        imu->due_ms = now + 100U;
        imu->config_index = 0;
        return;
    }
    if (imu->state == 2U)
    {
        unsigned i = imu->config_index;
        uint8_t readback = 0;
        if (!bus_ok(imu, now, C30D_I2cWrite(ADDRESS, 0x7f, (uint8_t)(config[i].bank << 4))))
        {
            return;
        }
        if (!bus_ok(imu, now, C30D_I2cWrite(ADDRESS, config[i].reg, config[i].value)))
        {
            return;
        }
        if (!bus_ok(imu, now, C30D_I2cRead(ADDRESS, config[i].reg, &readback, 1)))
        {
            return;
        }
        if (readback != config[i].value)
        {
            ++d->config_errors;
            failed(imu, now, C30D_IMU_CONFIG_ERROR, C30D_I2C_OK);
            return;
        }
        ++imu->config_index;
        if (imu->config_index == sizeof(config) / sizeof(config[0]))
        {
            imu->state = 3;
            imu->ready_since_ms = now;
        }
        return;
    }
    uint8_t ready = 0;
    if (!bus_ok(imu, now, C30D_I2cRead(ADDRESS, 0x1a, &ready, 1)))
    {
        return;
    }
#ifdef C30D_IMU_TEST_HOOKS
    if ((c30d_imu_test_fault & 4U) != 0U)
    {
        ready = 0;
    }
#endif
    if ((ready & 1U) == 0U)
    {
        ++d->not_ready_count;
        uint32_t last = d->valid ? d->timestamp_ms : imu->ready_since_ms;
        if ((uint32_t)(now - last) >= STALE_MS)
        {
            ++d->stale_count;
            failed(imu, now, C30D_IMU_STALE, C30D_I2C_OK);
        }
        return;
    }
    uint8_t bytes[14];
    if (!bus_ok(imu, now, C30D_I2cRead(ADDRESS, 0x2d, bytes, sizeof(bytes))))
    {
        return;
    }
    for (unsigned i = 0; i < 3U; ++i)
    {
        d->accel_raw[i] = signed_word(&bytes[i * 2U]);
        d->gyro_raw[i] = signed_word(&bytes[6U + i * 2U]);
        d->accel_m_s2[i] = (float)d->accel_raw[i] * (9.80665f / 16384.0f);
        d->gyro_rad_s[i] = (float)d->gyro_raw[i] * (0.0174532925199433f / 131.0f);
    }
    d->temperature_raw = signed_word(&bytes[12]);
    d->temperature_c = (float)d->temperature_raw / 333.87f + 21.0f;
    if (imu->last_sample_cycles != 0U)
    {
        d->period_us = C30D_ElapsedUs(imu->last_sample_cycles);
        range(d->period_us, &d->period_min_us, &d->period_max_us);
    }
    imu->last_sample_cycles = C30D_Cycles();
    d->timestamp_ms = now;
    ++d->sequence;
    d->valid = 1;
    d->health = C30D_IMU_OK;
    if (imu->had_error != 0U)
    {
        ++d->recovery_count;
        imu->had_error = 0;
    }
}
