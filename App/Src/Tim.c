#include "Tim.h"
#include "stm32f1xx.h"
void Tim2_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->CR1 = TIM_CR1_OPM;
    TIM2->DIER = 0U;
    TIM2->PSC = 63U; /* APB1 timer clock = 64 MHz, counter = 1 MHz. */
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    NVIC_SetPriority(TIM2_IRQn, 5U);
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
}
void Tim2_Start(uint16_t microseconds)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->CNT = 0U;
    TIM2->ARR = microseconds ? microseconds - 1U : 0U;
    TIM2->SR = 0U;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
}
void Tim2_Stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
    TIM2->DIER = 0U;
    TIM2->SR = 0U;
    TIM2->CNT = 0U;
    NVIC_ClearPendingIRQ(TIM2_IRQn);
}
