/* Host tests of real driver state machine with an emulated register transport.
 * Does not replace hardware tests of I2C timing or panel wiring. */
#include "c30d_imu.h"
#include "c30d_soft_i2c.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint8_t regs[3][128], bank;
static uint32_t cycles;
static int fail_read, corrupt_config;
volatile uint32_t c30d_imu_test_fault;
uint32_t C30D_Cycles(void) { return cycles; }
uint32_t C30D_ElapsedUs(uint32_t start) { return (uint32_t)(cycles - start) / 168U; }
void C30D_I2cInit(void) {}
C30D_I2cStatus C30D_I2cRecover(void) { return C30D_I2C_OK; }
C30D_I2cStatus C30D_I2cRead(uint8_t a, uint8_t r, uint8_t *data, size_t n)
{
    assert(a == 0x68 && n > 0 && n <= 14 && r + n <= 128);
    if (fail_read)
    {
        return C30D_I2C_NACK;
    }
    memcpy(data, &regs[bank][r], n);
    if (corrupt_config && bank == 2 && r == 1)
    {
        data[0] ^= 1;
    }
    return C30D_I2C_OK;
}
C30D_I2cStatus C30D_I2cWrite(uint8_t a, uint8_t r, uint8_t v)
{
    assert(a == 0x68);
    if (r == 0x7f)
    {
        bank = v >> 4;
        assert(bank < 3);
    }
    else
    {
        regs[bank][r] = v;
    }
    return C30D_I2C_OK;
}
static void poll(C30D_Imu *imu, uint32_t now)
{
    cycles = now * 168000U;
    C30D_ImuPoll(imu, now);
}
static void reset(C30D_Imu *imu)
{
    memset(regs, 0, sizeof(regs));
    bank = 0;
    fail_read = 0;
    corrupt_config = 0;
    c30d_imu_test_fault = 0;
    regs[0][0] = 0xea;
    regs[0][0x1a] = 1;
    regs[0][0x2d] = 0x40; /* ax=+1g */
    regs[0][0x2f] = 0x80; /* ay=-2g */
    regs[0][0x33] = 0xff;
    regs[0][0x34] = 0x7d; /* gx=-131 => -1deg/s */
    C30D_ImuInit(imu);
}
static void boot(C30D_Imu *imu)
{
    for (uint32_t t = 0; t <= 400; t += 10)
    {
        poll(imu, t);
    }
}
int main(void)
{
    C30D_Imu imu;
    reset(&imu);
    boot(&imu);
    assert(imu.data.valid && imu.data.health == C30D_IMU_OK);
    assert(imu.data.accel_raw[0] == 16384 && imu.data.accel_raw[1] == -32768);
    assert(imu.data.gyro_raw[0] == -131);
    assert(fabsf(imu.data.accel_m_s2[0] - 9.80665f) < 0.00001f);
    assert(fabsf(imu.data.gyro_rad_s[0] + 0.0174532925f) < 0.000001f);
    assert(imu.data.temperature_c == 21.0f && imu.data.period_us == 10000);
    uint32_t n = imu.data.sequence;
    fail_read = 1;
    poll(&imu, 410);
    assert(!imu.data.valid && imu.data.nack_count == 1);
    poll(&imu, 420);
    assert(imu.data.sequence == n && imu.data.retry_count == 1);
    fail_read = 0;
    for (uint32_t t = 1410; t <= 1810; t += 10)
    {
        poll(&imu, t);
    }
    assert(imu.data.valid && imu.data.recovery_count == 1 && imu.data.last_error == C30D_IMU_BUS_ERROR);
    regs[0][0x1a] = 0;
    poll(&imu, 1820);
    poll(&imu, 1830);
    poll(&imu, 1840);
    assert(!imu.data.valid && imu.data.health == C30D_IMU_STALE && imu.data.stale_count == 1);
    reset(&imu);
    regs[0][0] = 0;
    boot(&imu);
    assert(!imu.data.valid && imu.data.id_errors == 1 && imu.data.health == C30D_IMU_ID_ERROR);
    reset(&imu);
    corrupt_config = 1;
    boot(&imu);
    assert(!imu.data.valid && imu.data.config_errors == 1 && imu.data.health == C30D_IMU_CONFIG_ERROR);
    reset(&imu);
    boot(&imu);
    n = imu.data.sequence;
    poll(&imu, 0x80001000U);
    assert(imu.data.sequence == n + 1); /* old due_ms must not gate running acquisition */
    poll(&imu, 0xfffffff5U);
    n = imu.data.sequence;
    poll(&imu, 5U);
    assert(imu.data.sequence == n + 1);
    reset(&imu);
    poll(&imu, 0xfffffff0U);
    assert(imu.state == 1); /* start near tick wrap */
    poll(&imu, 20U);
    assert(imu.state == 1);
    poll(&imu, 84U);
    assert(imu.state == 2);
    puts("PASS: initialization/readback, signed conversion/SI, 10ms period, NACK/backoff/recovery, stale, bad ID/config, tick wrap");
}
