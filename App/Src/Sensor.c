#include "Sensor.h"
#include "Health.h"
#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

#define ADC_BUFFER_SIZE 32
static volatile uint16_t adc_buffer[ADC_BUFFER_SIZE];
static volatile uint16_t sample_raw;
static volatile uint32_t sample_sequence;
static volatile bool sample_error;

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

    TIM3->PSC = 63;
    TIM3->ARR = 999; /* 1 kHz; each DMA half contains 16 ms of samples. */

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

    ADC1->SQR1 = 0U;
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = 0U;

    ADC1->SMPR2 = 0;
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0;
}
void Sensor_ADC_Start(void)
{
    ADC1->CR2 |= ADC_CR2_ADON;

    // РљРѕСЂРѕС‚РєР°СЏ Р·Р°РґРµСЂР¶РєР° РїРѕСЃР»Рµ РІРєР»СЋС‡РµРЅРёСЏ ADC
    for (volatile int i = 0; i < 10000; i++)
    {
        __NOP();
    }

    ADC1->CR2 |= ADC_CR2_RSTCAL;
    uint32_t limit = 1000000U;
    while (ADC1->CR2 & ADC_CR2_RSTCAL) if (--limit == 0U) Error_Handler();

    ADC1->CR2 |= ADC_CR2_CAL;
    limit = 1000000U;
    while (ADC1->CR2 & ADC_CR2_CAL) if (--limit == 0U) Error_Handler();
}
void Sensor_ADC_Stop(void)
{
    ADC1->CR2 &= ~ADC_CR2_ADON;
}

void Sensor_DMA_Init(void)
{
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel1->CCR = 0;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    sample_error = false;
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

/* The ISR averages only a completed half. No task ever reads ADC1->DR. */
void Sensor_DMA_IRQHandler(void)
{
    uint32_t flags = DMA1->ISR;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    if (flags & DMA_ISR_TEIF1) {
        sample_error = true;
        Health_Increment(COUNT_ADC_DMA_ERRORS);
        Sensor_TIM3_Stop();
        Sensor_DMA_Stop();
        return;
    }
    if (!(flags & (DMA_ISR_HTIF1 | DMA_ISR_TCIF1))) return;
    /* CNDTR identifies the half not currently being written, including when
     * both flags accumulated while Flash stalled instruction fetching. */
    uint32_t offset = DMA1_Channel1->CNDTR <= ADC_BUFFER_SIZE / 2U ? 0U : ADC_BUFFER_SIZE / 2U;
    uint32_t sum = 0U;
    for (uint32_t i = 0U; i < ADC_BUFFER_SIZE / 2U; ++i) sum += adc_buffer[offset + i];
    sample_raw = (uint16_t)((sum + ADC_BUFFER_SIZE / 4U) / (ADC_BUFFER_SIZE / 2U));
    ++sample_sequence;
}

void Sensor_Read(uint16_t *raw, uint32_t *sequence, bool *dma_error)
{
    taskENTER_CRITICAL();
    *raw = sample_raw;
    *sequence = sample_sequence;
    *dma_error = sample_error;
    taskEXIT_CRITICAL();
}

void Sensor_Recover(void)
{
    Health_Increment(COUNT_ADC_RESTARTS);
    Sensor_TIM3_Stop();
    Sensor_DMA_Stop();
    NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    Sensor_ADC_Stop();
    Sensor_ADC_Init();
    Sensor_ADC_Start();
    Sensor_DMA_Init();
    NVIC_ClearPendingIRQ(DMA1_Channel1_IRQn);
    Sensor_DMA_Start();
    Sensor_TIM3_Start();
}
