#include "Tim.h"
#include "stm32f103xb.h"
#include "stm32f1xx.h"
void Tim3_Init(void)
{
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->SMCR = 0;
    TIM3->DIER = 0;

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CR2 &= ~TIM_CR2_MMS;

    TIM3->CR2 |= TIM_CR2_MMS_1;

    TIM3->CNT = 0;

    TIM3->PSC = 6399;
    TIM3->ARR = 99;

    TIM3->EGR = TIM_EGR_UG;
    TIM3->SR &= ~TIM_SR_UIF;

    TIM3->DIER = 0;

    TIM3->CR1 |= TIM_CR1_ARPE;
}

void Tim3_Start(void)
{
    TIM3->CR1 |= TIM_CR1_CEN;
}

void Tim3_Stop(void)
{
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

void Tim2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->CR1 |= TIM_CR1_OPM;

    TIM2->PSC = 63U;

    TIM2->EGR = TIM_EGR_UG;

    TIM2->SR &= ~TIM_SR_UIF;
}

void Tim2_Start(uint16_t arr)
{
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CNT = 0U;

    TIM2->ARR = arr;

    TIM2->SR &= ~TIM_SR_UIF;

    TIM2->CR1 |= TIM_CR1_CEN;
}

void Tim2_Stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT = 0U;
}