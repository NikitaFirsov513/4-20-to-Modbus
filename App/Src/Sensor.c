#include "Sensor.h"
#include "stm32f1xx.h"

#define ADC_BUFFER_SIZE 32
uint16_t adc_buffer[ADC_BUFFER_SIZE];

void Sensor_TIM3_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->CR1 = 0;
    TIM3->CR2 = 0;
    TIM3->SMCR = 0;
    TIM3->DIER = 0;

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

void Sensor_TIM3_Start(void)
{
    TIM3->CR1 |= TIM_CR1_CEN;
}

void Sensor_TIM3_Stop(void)
{
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

void Sensor_ADC_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    ADC1->CR1 = 0;
    ADC1->CR2 = 0;

    ADC1->CR2 |= ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_EXTSEL_2;
    ADC1->CR2 |= ADC_CR2_EXTTRIG;

    ADC1->SQR1 &= ~ADC_SQR1_L;
    ADC1->SQR3 &= ~ADC_SQR3_SQ1;

    ADC1->SMPR2 = 0;
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0;
}
void Sensor_ADC_Start(void)
{
    ADC1->CR2 |= ADC_CR2_ADON;

    // Короткая задержка после включения ADC
    for (volatile int i = 0; i < 10000; i++)
    {
        __NOP();
    }

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    while (ADC1->CR2 & ADC_CR2_RSTCAL)
    {
    }

    ADC1->CR2 |= ADC_CR2_CAL;
    while (ADC1->CR2 & ADC_CR2_CAL)
    {
    }
}
void Sensor_ADC_Stop(void)
{
    ADC1->CR2 &= ~ADC_CR2_ADON;
}

void Sensor_DMA_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel1->CCR = 0;
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;
    DMA1_Channel1->CCR |= DMA_CCR_HTIE;
    DMA1_Channel1->CCR |= DMA_CCR_TEIE;
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;
    DMA1_Channel1->CCR |= DMA_CCR_MINC;
    DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel1->CCR |= DMA_CCR_PL_1 | DMA_CCR_PL_0;

    NVIC_SetPriority(DMA1_Channel1_IRQn, 5);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    DMA1_Channel1->CNDTR = ADC_BUFFER_SIZE;

    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
    DMA1_Channel1->CMAR = (uint32_t)adc_buffer;
}
void Sensor_DMA_Start(void)
{
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;

    DMA1->IFCR = DMA_IFCR_CGIF1 |
                 DMA_IFCR_CTCIF1 |
                 DMA_IFCR_CHTIF1 |
                 DMA_IFCR_CTEIF1;

    DMA1_Channel1->CNDTR = ADC_BUFFER_SIZE;

    DMA1_Channel1->CCR |= DMA_CCR_EN;
}
void Sensor_DMA_Stop(void)
{
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
}

void Sensor_DMA_IRQHandler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        // Обработка события завершения передачи
    }

    if (DMA1->ISR & DMA_ISR_HTIF1)
    {
        DMA1->IFCR |= DMA_IFCR_CHTIF1;
        // Обработка события половины передачи

    }

    if (DMA1->ISR & DMA_ISR_TEIF1)
    {
        DMA1->IFCR |= DMA_IFCR_CTEIF1;
        // Обработка события ошибки передачи
        
    }
}