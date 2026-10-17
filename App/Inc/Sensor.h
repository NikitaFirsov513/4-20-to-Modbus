#pragma once
#include "stm32f103xb.h"

void Sensor_TIM3_Init(void);
void Sensor_TIM3_Start(void);
void Sensor_TIM3_Stop(void);

void Sensor_ADC_Init(void);
void Sensor_ADC_Start(void);
void Sensor_ADC_Stop(void);

void Sensor_DMA_Init(void);
void Sensor_DMA_Start(void);
void Sensor_DMA_Stop(void);

void Sensor_DMA_IRQHandler(void);

#include <stdbool.h>
void Sensor_Read(uint16_t *raw, uint32_t *sequence, bool *dma_error);

void Sensor_Recover(void);
