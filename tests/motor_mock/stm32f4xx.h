#ifndef MOTOR_MOCK_STM32_H
#define MOTOR_MOCK_STM32_H
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
#endif
    typedef struct
    {
        volatile uint32_t CR1, SMCR, CCER, CCMR1, CCMR2, ARR, PSC, CNT, EGR;
        volatile uint32_t CCR1, CCR2, CCR3, CCR4, BDTR;
    } TIM_TypeDef;
    typedef struct
    {
        volatile uint32_t APB1ENR, APB2ENR;
    } RCC_TypeDef;
    extern TIM_TypeDef mock_tim[9];
    extern RCC_TypeDef mock_rcc;
    extern uint32_t mock_primask;
#define TIM1 (&mock_tim[0])
#define TIM2 (&mock_tim[1])
#define TIM3 (&mock_tim[2])
#define TIM4 (&mock_tim[3])
#define TIM5 (&mock_tim[4])
#define TIM9 (&mock_tim[5])
#define TIM10 (&mock_tim[6])
#define TIM11 (&mock_tim[7])
#define RCC (&mock_rcc)
#define RCC_APB2ENR_TIM1EN (1U << 0)
#define RCC_APB2ENR_TIM9EN (1U << 16)
#define RCC_APB2ENR_TIM10EN (1U << 17)
#define RCC_APB2ENR_TIM11EN (1U << 18)
#define RCC_APB1ENR_TIM2EN (1U << 0)
#define RCC_APB1ENR_TIM3EN (1U << 1)
#define RCC_APB1ENR_TIM4EN (1U << 2)
#define RCC_APB1ENR_TIM5EN (1U << 3)
#define TIM_BDTR_MOE (1U << 15)
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC2E (1U << 4)
#define TIM_CR1_CEN 1U
#define TIM_CCMR1_OC1PE (1U << 3)
    static inline uint32_t __get_PRIMASK(void) { return mock_primask; }
    static inline void __disable_irq(void) { mock_primask = 1U; }
    static inline void __set_PRIMASK(uint32_t value) { mock_primask = value; }
#ifdef __cplusplus
}
#endif
#endif
