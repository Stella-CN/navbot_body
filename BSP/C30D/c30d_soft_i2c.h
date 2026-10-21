#ifndef C30D_SOFT_I2C_H
#define C30D_SOFT_I2C_H
#include <stdint.h>
#include <stddef.h>

typedef enum
{
    C30D_I2C_OK,
    C30D_I2C_NACK,
    C30D_I2C_TIMEOUT,
    C30D_I2C_STUCK
} C30D_I2cStatus;
/* Single owner: sensor task. No ISR access. */
void C30D_I2cInit(void);
C30D_I2cStatus C30D_I2cRecover(void);
C30D_I2cStatus C30D_I2cRead(uint8_t address, uint8_t reg, uint8_t *data, size_t size);
C30D_I2cStatus C30D_I2cWrite(uint8_t address, uint8_t reg, uint8_t value);
uint32_t C30D_Cycles(void);
uint32_t C30D_ElapsedUs(uint32_t start);
#ifdef C30D_IMU_TEST_HOOKS
/* Debugger-only bench injection: 1=real address NACK, 2=simulated SCL-low,
 * 4=suppress data-ready. Zero by default; absent from Release builds. */
extern volatile uint32_t c30d_imu_test_fault;
#endif
#endif
