#include "c30d_safety.h"
#include "stm32f4xx.h"

/* Template interlock: there is deliberately no motor-enable API.
 * Register access also works before HAL handle initialization. Only touch
 * peripherals whose clocks are enabled, including early error paths. */
void C30D_ForceMotorIdle(void)
{
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
        TIM1->CCER = 0U;
        TIM1->CCR1 = 0U;
        TIM1->CCR2 = 0U;
        TIM1->CCR3 = 0U;
        TIM1->CCR4 = 0U;
    }
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM9EN) != 0U)
    {
        TIM9->CCER = 0U;
        TIM9->CCR1 = 0U;
        TIM9->CCR2 = 0U;
    }
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM10EN) != 0U)
    {
        TIM10->CCER = 0U;
        TIM10->CCR1 = 0U;
    }
    if ((RCC->APB2ENR & RCC_APB2ENR_TIM11EN) != 0U)
    {
        TIM11->CCER = 0U;
        TIM11->CCR1 = 0U;
    }
}

/* Faults latch until reset. This is a software output inhibition only. */
_Noreturn void C30D_TemplatePanic(void)
{
    __disable_irq();
    C30D_ForceMotorIdle();
    for (;;)
    {
        __WFI();
    }
}

