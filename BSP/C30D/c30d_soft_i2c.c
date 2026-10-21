#include "c30d_soft_i2c.h"
#include "stm32f4xx_hal.h"

#define SCL (1U << 10)
#define SDA (1U << 11)
#define TRANSACTION_US 2500U
static uint32_t transaction_start;
#ifdef C30D_IMU_TEST_HOOKS
volatile uint32_t c30d_imu_test_fault;
#endif
uint32_t C30D_Cycles(void) { return DWT->CYCCNT; }
uint32_t C30D_ElapsedUs(uint32_t start) { return (uint32_t)(C30D_Cycles() - start) / (SystemCoreClock / 1000000U); }
static void delay_half(void)
{
    uint32_t t = C30D_Cycles();
    while (C30D_ElapsedUs(t) < 3U)
    {
    }
}
static void line(uint32_t pin, int high) { GPIOB->BSRR = high ? pin : pin << 16; }
static C30D_I2cStatus clock_high(void)
{
    line(SCL, 1);
    uint32_t t = C30D_Cycles();
    for (;;)
    {
        int high = (GPIOB->IDR & SCL) != 0U;
#ifdef C30D_IMU_TEST_HOOKS
        if ((c30d_imu_test_fault & 2U) != 0U)
        {
            high = 0;
        }
#endif
        if (C30D_ElapsedUs(transaction_start) >= TRANSACTION_US || C30D_ElapsedUs(t) >= 100U)
        {
            return C30D_I2C_TIMEOUT;
        }
        if (high)
        {
            delay_half();
            return C30D_I2C_OK;
        }
    }
}
static C30D_I2cStatus stop(void)
{
    line(SCL, 0);
    line(SDA, 0);
    delay_half();
    C30D_I2cStatus result = clock_high();
    line(SDA, 1);
    delay_half();
    /* Always release both wires on failure as well. */
    line(SCL, 1);
    return result;
}
static C30D_I2cStatus start(void)
{
    line(SDA, 1);
    delay_half();
    C30D_I2cStatus result = clock_high();
    if (result != C30D_I2C_OK)
    {
        return result;
    }
    if ((GPIOB->IDR & SDA) == 0U)
    {
        return C30D_I2C_STUCK;
    }
    line(SDA, 0);
    delay_half();
    line(SCL, 0);
    return C30D_I2C_OK;
}
static C30D_I2cStatus send(uint8_t value)
{
    for (unsigned i = 0; i < 8U; ++i)
    {
        line(SDA, (value & 0x80U) != 0U);
        delay_half();
        C30D_I2cStatus result = clock_high();
        if (result != C30D_I2C_OK)
        {
            return result;
        }
        line(SCL, 0);
        value <<= 1;
    }
    line(SDA, 1);
    delay_half();
    C30D_I2cStatus result = clock_high();
    if (result != C30D_I2C_OK)
    {
        return result;
    }
    int nack = (GPIOB->IDR & SDA) != 0U;
    line(SCL, 0);
    return nack ? C30D_I2C_NACK : C30D_I2C_OK;
}
static C30D_I2cStatus receive(uint8_t *value, int last)
{
    uint8_t data = 0;
    line(SDA, 1);
    for (unsigned i = 0; i < 8U; ++i)
    {
        delay_half();
        C30D_I2cStatus result = clock_high();
        if (result != C30D_I2C_OK)
        {
            return result;
        }
        data = (uint8_t)((data << 1) | ((GPIOB->IDR & SDA) != 0U));
        line(SCL, 0);
    }
    line(SDA, last);
    delay_half();
    C30D_I2cStatus result = clock_high();
    line(SCL, 0);
    line(SDA, 1);
    if (result == C30D_I2C_OK)
    {
        *value = data;
    }
    return result;
}
static uint8_t device(uint8_t address)
{
#ifdef C30D_IMU_TEST_HOOKS
    if ((c30d_imu_test_fault & 1U) != 0U)
    {
        return 0x69U;
    }
#endif
    return address;
}
void C30D_I2cInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIOB->BSRR = SCL | SDA;
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = SCL | SDA;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
}
C30D_I2cStatus C30D_I2cRecover(void)
{
    transaction_start = C30D_Cycles();
    line(SDA, 1);
    for (unsigned i = 0; i < 9U; ++i)
    {
        line(SCL, 0);
        delay_half();
        C30D_I2cStatus result = clock_high();
        if (result != C30D_I2C_OK)
        {
            (void)stop();
            return result;
        }
    }
    C30D_I2cStatus result = stop();
    if (result == C30D_I2C_OK && (GPIOB->IDR & SDA) == 0U)
    {
        result = C30D_I2C_STUCK;
    }
    return result;
}
C30D_I2cStatus C30D_I2cRead(uint8_t address, uint8_t reg, uint8_t *data, size_t size)
{
    if (data == NULL || size == 0U || size > 14U)
    {
        return C30D_I2C_STUCK;
    }
    transaction_start = C30D_Cycles();
    C30D_I2cStatus result = start();
    if (result == C30D_I2C_OK)
    {
        result = send((uint8_t)(device(address) << 1));
    }
    if (result == C30D_I2C_OK)
    {
        result = send(reg);
    }
    if (result == C30D_I2C_OK)
    {
        result = start();
    }
    if (result == C30D_I2C_OK)
    {
        result = send((uint8_t)((device(address) << 1) | 1U));
    }
    for (size_t i = 0; result == C30D_I2C_OK && i < size; ++i)
    {
        result = receive(&data[i], i + 1U == size);
    }
    C30D_I2cStatus end = stop();
    return result == C30D_I2C_OK ? end : result;
}
C30D_I2cStatus C30D_I2cWrite(uint8_t address, uint8_t reg, uint8_t value)
{
    transaction_start = C30D_Cycles();
    C30D_I2cStatus result = start();
    if (result == C30D_I2C_OK)
    {
        result = send((uint8_t)(device(address) << 1));
    }
    if (result == C30D_I2C_OK)
    {
        result = send(reg);
    }
    if (result == C30D_I2C_OK)
    {
        result = send(value);
    }
    C30D_I2cStatus end = stop();
    return result == C30D_I2C_OK ? end : result;
}
