#include "c30d_motor.h"
#include "stm32f4xx.h"

typedef struct
{
    TIM_TypeDef *first, *second, *encoder;
    uint32_t first_channel, second_channel, pwm_clocks, encoder_clock;
} MotorPins;

static int pins(C30D_MotorChannel channel, MotorPins *p)
{
    switch (channel)
    {
    case C30D_MOTOR_A:
        *p = (MotorPins){TIM10, TIM11, TIM2, 0U, 0U, RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN, RCC_APB1ENR_TIM2EN};
        break;
    case C30D_MOTOR_B:
        *p = (MotorPins){TIM9, TIM9, TIM3, 0U, 1U, RCC_APB2ENR_TIM9EN, RCC_APB1ENR_TIM3EN};
        break;
    case C30D_MOTOR_C:
        *p = (MotorPins){TIM1, TIM1, TIM4, 0U, 1U, RCC_APB2ENR_TIM1EN, RCC_APB1ENR_TIM4EN};
        break;
    case C30D_MOTOR_D:
        *p = (MotorPins){TIM1, TIM1, TIM5, 2U, 3U, RCC_APB2ENR_TIM1EN, RCC_APB1ENR_TIM5EN};
        break;
    default:
        return 0;
    }
    return 1;
}

static int clocks_ready(const MotorPins *p)
{
    return (RCC->APB2ENR & p->pwm_clocks) == p->pwm_clocks &&
           (RCC->APB1ENR & p->encoder_clock) == p->encoder_clock;
}

static int timer_clock(TIM_TypeDef *timer)
{
    uint32_t mask = timer == TIM1 ? RCC_APB2ENR_TIM1EN : timer == TIM9 ? RCC_APB2ENR_TIM9EN
                                                     : timer == TIM10  ? RCC_APB2ENR_TIM10EN
                                                                       : RCC_APB2ENR_TIM11EN;
    return (RCC->APB2ENR & mask) != 0U;
}

static int pwm_configured(TIM_TypeDef *timer, uint32_t channel)
{
    uint32_t mode = channel < 2U ? timer->CCMR1 : timer->CCMR2;
    /* PWM1, output mode, edge-aligned upcounting at 168MHz / 16800. */
    return timer->ARR == 16799U && timer->PSC == 0U && (timer->CR1 & 0x70U) == 0U &&
           ((mode >> (8U * (channel % 2U))) & 0x73U) == 0x60U;
}

static volatile uint32_t *compare(TIM_TypeDef *timer, uint32_t channel)
{
    switch (channel)
    {
    case 0U:
        return &timer->CCR1;
    case 1U:
        return &timer->CCR2;
    case 2U:
        return &timer->CCR3;
    default:
        return &timer->CCR4;
    }
}

static void disable_pair(const MotorPins *p)
{
    p->first->CCER &= ~(0xFU << (4U * p->first_channel));
    p->second->CCER &= ~(0xFU << (4U * p->second_channel));
    if (p->first == TIM1 && (TIM1->CCER & 0x1111U) == 0U)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
}

static void immediate_compare(TIM_TypeDef *timer, uint32_t channel, uint32_t value)
{
    /* Pair is disabled first. Remove only its preload bit to update immediately;
     * never issue shared TIM1 UG or change CNT/ARR while another wheel runs. */
    volatile uint32_t *mode = channel < 2U ? &timer->CCMR1 : &timer->CCMR2;
    *mode &= ~(TIM_CCMR1_OC1PE << (8U * (channel % 2U)));
    *compare(timer, channel) = value;
}

static void apply(const MotorPins *p, uint32_t first, uint32_t second, int enabled)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    disable_pair(p);
    immediate_compare(p->first, p->first_channel, first);
    immediate_compare(p->second, p->second_channel, second);
    if (enabled)
    {
        p->first->CCER |= TIM_CCER_CC1E << (4U * p->first_channel);
        p->second->CCER |= TIM_CCER_CC1E << (4U * p->second_channel);
        if (p->first == TIM1)
        {
            TIM1->BDTR |= TIM_BDTR_MOE;
        }
        p->first->CR1 |= TIM_CR1_CEN;
        p->second->CR1 |= TIM_CR1_CEN;
    }
    __set_PRIMASK(irq);
}

void C30D_MotorCoast(C30D_MotorChannel channel)
{
    MotorPins p;
    if (!pins(channel, &p))
    {
        return;
    }
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    /* Inhibit any accessible half even if an encoder or the other PWM clock
     * is missing. Stopping must never depend on encoder availability. */
    if (timer_clock(p.first))
    {
        p.first->CCER &= ~(0xFU << (4U * p.first_channel));
        immediate_compare(p.first, p.first_channel, 0U);
    }
    if (timer_clock(p.second))
    {
        p.second->CCER &= ~(0xFU << (4U * p.second_channel));
        immediate_compare(p.second, p.second_channel, 0U);
    }
    if (p.first == TIM1 && timer_clock(TIM1) && (TIM1->CCER & 0x1111U) == 0U)
    {
        TIM1->BDTR &= ~TIM_BDTR_MOE;
    }
    __set_PRIMASK(irq);
}

int C30D_MotorInit(C30D_MotorChannel channel)
{
    MotorPins p;
    C30D_MotorCoast(channel);
    if (!pins(channel, &p) || !clocks_ready(&p))
    {
        return 0;
    }
    uint32_t encoder_mask = channel == C30D_MOTOR_B || channel == C30D_MOTOR_C ? 0xffffU : 0xffffffffU;
    if (!pwm_configured(p.first, p.first_channel) || !pwm_configured(p.second, p.second_channel) ||
        (p.encoder->SMCR & 7U) != 3U || p.encoder->ARR != encoder_mask)
    {
        return 0;
    }
    p.encoder->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
    p.encoder->CR1 |= TIM_CR1_CEN;
    return 1;
}

int C30D_MotorWrite(C30D_MotorChannel channel, int32_t duty)
{
    MotorPins p;
    if (!pins(channel, &p))
    {
        return 0;
    }
    if (!clocks_ready(&p) || duty < -1000 || duty > 1000 ||
        !pwm_configured(p.first, p.first_channel) || !pwm_configured(p.second, p.second_channel))
    {
        C30D_MotorCoast(channel);
        return 0;
    }
    if (duty == 0)
    {
        apply(&p, 0U, 0U, 0);
        return 1;
    }
    uint32_t magnitude = (uint32_t)(duty < 0 ? -duty : duty);
    uint32_t low = 16800U - 16800U * magnitude / 1000U;
    apply(&p, duty > 0 ? 16800U : low, duty > 0 ? low : 16800U, 1);
    return 1;
}

int C30D_MotorBrake(C30D_MotorChannel channel)
{
    MotorPins p;
    if (!pins(channel, &p))
    {
        return 0;
    }
    if (!clocks_ready(&p) || !pwm_configured(p.first, p.first_channel) || !pwm_configured(p.second, p.second_channel))
    {
        C30D_MotorCoast(channel);
        return 0;
    }
    apply(&p, 16800U, 16800U, 1);
    return 1;
}

uint32_t C30D_MotorCounter(C30D_MotorChannel channel)
{
    MotorPins p;
    return pins(channel, &p) && clocks_ready(&p) ? p.encoder->CNT : 0U;
}
