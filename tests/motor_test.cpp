#include "motor.hpp"
#include "wheel_motors.hpp"
#include "stm32f4xx.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <initializer_list>
#include <type_traits>

TIM_TypeDef mock_tim[9];
RCC_TypeDef mock_rcc;
uint32_t mock_primask;
using namespace navbot;

class FakeMotor final : public Motor
{
public:
    explicit FakeMotor(uint8_t bits = 16)
        : Motor({WheelPosition::FrontLeft, 'C', "test", 12000, 10, 500, 4, bits, false}) {}
    uint32_t counter{};
    int32_t output{};
    bool hardware_ok{true}, braking{};

private:
    bool hardwareInit() override { return hardware_ok; }
    bool hardwareWrite(int32_t duty) override
    {
        output = duty;
        braking = false;
        return hardware_ok;
    }
    void hardwareCoast() override
    {
        output = 0;
        braking = false;
    }
    bool hardwareBrake() override
    {
        braking = true;
        return hardware_ok;
    }
    uint32_t hardwareCounter() const override { return counter; }
};

static void lifecycle()
{
    FakeMotor m;
    assert(!m.enable());
    assert(!m.setDuty(100) && m.output == 0);
    assert(m.initialize(0));
    assert(!m.enable()); // Need a fresh feedback baseline before arming.
    assert(m.updateFeedback(10) && m.enable());
    assert(m.setDuty(200) && m.output == 200);
    assert(!m.configure(300, 1, 1) && m.output == 200);
    assert(m.setDuty(-200) && m.output == -200);
    assert(!m.setDuty(201) && m.output == 0 && m.state() == MotorState::Disabled);
    assert(!m.enable()); // Error remains latched until initialize.
    assert(m.initialize(20) && m.updateFeedback(30) && m.enable());
    assert(m.brake() && m.braking);
    m.coast();
    assert(!m.braking && m.output == 0 && m.state() == MotorState::Enabled);
    assert(m.setDuty(100));
    assert(m.setDuty(0) && m.output == 0);
    assert(!m.setDuty(std::numeric_limits<int32_t>::min()) && m.output == 0);
    assert(!m.configure(0, 1, 1));
    assert(!m.configure(1001, 1, 1));
    assert(!m.configure(100, 0, 1));
    assert(m.configure(1000, -1, -1));
    assert(!m.enable());
    assert(m.initialize(50));
    m.counter = 200;
    assert(m.updateFeedback(60) && m.feedback().delta_counts == -200);
    assert(std::fabs(m.feedback().rpm + 60.0f) < 0.001f);
    assert(m.enable() && m.setDuty(1000) && m.output == -1000);
    m.hardware_ok = false;
    assert(!m.setDuty(100) && m.output == 0);
    assert(!m.initialize(70) && m.state() == MotorState::Uninitialized);
    FakeMotor invalid(8);
    assert(!invalid.initialize(0));
}

static void feedback()
{
    for (uint8_t bits : {uint8_t(16), uint8_t(32)})
    {
        FakeMotor m(bits);
        uint32_t mask = bits == 16 ? 0xffffU : 0xffffffffU;
        m.counter = mask - 4;
        assert(m.initialize(0xfffffff8U));
        m.counter = 5;
        assert(m.updateFeedback(2)); // Counter and millisecond wrap together.
        assert(m.feedback().delta_counts == 10 && m.feedback().total_counts == 10);
        assert(m.enable() && m.setDuty(100));
        m.counter = mask - 4;
        assert(m.updateFeedback(12));
        assert(m.feedback().delta_counts == -10 && m.feedback().total_counts == 0);
        assert(!m.updateFeedback(12) && m.output == 0 && !m.feedback().valid);
        assert(m.error() == MotorError::SampleInterval);
        assert(m.initialize(20) && m.updateFeedback(30) && m.enable());
        assert(m.setDuty(100));
        assert(!m.updateFeedback(51) && m.output == 0);
        assert(m.initialize(60));
        m.counter = (m.counter + (mask >> 1U) + 1U) & mask;
        assert(!m.updateFeedback(70) && m.error() == MotorError::CounterRange);
    }
    FakeMotor a, b;
    assert(a.initialize(0) && b.initialize(0));
    a.counter = 200;
    b.counter = 300;
    assert(a.updateFeedback(10) && b.updateFeedback(10));
    assert(a.feedback().total_counts == 200 && b.feedback().total_counts == 300);
    auto snapshot = a.feedback();
    snapshot.total_counts = 999;
    assert(a.feedback().total_counts == 200); // Value ownership, not aliasing.
}

static void setup_registers()
{
    for (auto &timer : mock_tim)
    {
        timer = {};
        timer.ARR = 16799;
        timer.SMCR = 3;
        timer.CCMR1 = timer.CCMR2 = 0x6868;
    }
    TIM2->ARR = TIM5->ARR = 0xffffffffU;
    TIM3->ARR = TIM4->ARR = 0xffffU;
    mock_rcc = {0xf, RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM9EN | RCC_APB2ENR_TIM10EN | RCC_APB2ENR_TIM11EN};
    mock_primask = 0;
}

static void registers_and_wheels()
{
    setup_registers();
    auto &wheels = WheelMotors::instance();
    assert(&wheels == &WheelMotors::instance());
    assert(wheels.frontLeft().info().connector == 'C');
    assert(wheels.frontRight().info().connector == 'B');
    assert(wheels.rearLeft().info().connector == 'D');
    assert(wheels.rearRight().info().connector == 'A');
    assert(wheels.frontLeft().info().countsPerRevolution() == 20000);
    assert(wheels.initialize(0));
    assert(TIM1->CCER == 0 && TIM9->CCER == 0 && TIM10->CCER == 0 && TIM11->CCER == 0);
    assert(TIM2->CR1 & TIM_CR1_CEN);
    assert(TIM3->CR1 & TIM_CR1_CEN);
    assert(TIM4->CR1 & TIM_CR1_CEN);
    assert(TIM5->CR1 & TIM_CR1_CEN);
    TIM2->CNT = 11;
    TIM3->CNT = 22;
    TIM4->CNT = 33;
    TIM5->CNT = 44;
    assert(wheels.updateFeedback(10));
    assert(wheels.rearRight().feedback().delta_counts == -11);
    assert(wheels.frontRight().feedback().delta_counts == 22);
    assert(wheels.frontLeft().feedback().delta_counts == 33);
    assert(wheels.rearLeft().feedback().delta_counts == -44);
    assert(wheels.frontLeft().enable() && wheels.rearLeft().enable());
    assert(wheels.frontLeft().setDuty(200));
    assert(wheels.rearLeft().setDuty(-100));
    assert(TIM1->CCR1 == 16800 && TIM1->CCR2 == 13440 && TIM1->CCR3 == 15120 && TIM1->CCR4 == 16800);
    assert(TIM1->CCER == 0x1111 && (TIM1->BDTR & TIM_BDTR_MOE));
    TIM1->CNT = 789;
    TIM1->EGR = 0x55;
    wheels.frontLeft().disable();
    assert(TIM1->CCER == 0x1100 && TIM1->CCR3 == 15120 && TIM1->CCR4 == 16800);
    assert((TIM1->BDTR & TIM_BDTR_MOE) && TIM1->CNT == 789 && TIM1->EGR == 0x55);
    assert(C30D_MotorInit(C30D_MOTOR_C)); // Reinitializing C must not disturb D.
    assert(TIM1->CCER == 0x1100 && TIM1->CCR3 == 15120 && (TIM1->BDTR & TIM_BDTR_MOE));
    wheels.rearLeft().disable();
    assert(TIM1->CCER == 0 && !(TIM1->BDTR & TIM_BDTR_MOE));
    assert(wheels.rearRight().enable() && wheels.frontRight().enable());
    assert(wheels.rearRight().setDuty(200));
    assert(TIM10->CCR1 == 16800 && TIM11->CCR1 == 13440);
    assert(wheels.frontRight().brake());
    assert(TIM9->CCR1 == 16800 && TIM9->CCR2 == 16800);
    assert(!wheels.updateFeedback(40)); // Late sample disables every wheel.
    assert(TIM10->CCER == 0 && TIM11->CCER == 0 && TIM9->CCER == 0);
    mock_primask = 1;
    assert(C30D_MotorWrite(C30D_MOTOR_A, 1000));
    assert(TIM10->CCR1 == 16800 && TIM11->CCR1 == 0 && mock_primask == 1);
    assert(!C30D_MotorWrite(C30D_MOTOR_A, std::numeric_limits<int32_t>::min()));
    assert(TIM10->CCER == 0 && TIM11->CCER == 0);
    assert(!C30D_MotorInit(static_cast<C30D_MotorChannel>(4)));
    TIM9->ARR = 100;
    assert(!C30D_MotorInit(C30D_MOTOR_B));
    mock_rcc.APB1ENR = 0;
    assert(!C30D_MotorInit(C30D_MOTOR_D));
    TIM10->CCER = TIM11->CCER = 1;
    TIM10->CCR1 = TIM11->CCR1 = 500;
    C30D_MotorCoast(C30D_MOTOR_A);
    assert(TIM10->CCER == 0 && TIM11->CCER == 0 && TIM10->CCR1 == 0 && TIM11->CCR1 == 0);
}

int main()
{
    static_assert(std::is_abstract<Motor>::value);
    static_assert(!std::is_copy_constructible<FrontLeftMotor>::value);
    static_assert(!std::is_move_constructible<RearRightMotor>::value);
    static_assert(std::is_base_of<Motor, FrontLeftMotor>::value && std::is_base_of<Motor, FrontRightMotor>::value);
    static_assert(std::is_base_of<Motor, RearLeftMotor>::value && std::is_base_of<Motor, RearRightMotor>::value);
    lifecycle();
    feedback();
    registers_and_wheels();
    std::puts("PASS: motor lifecycle, polarity, counter/tick wrap, faults, four-wheel mapping and TIM1 isolation");
}
