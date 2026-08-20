#ifndef TIM_H
#define TIM_H
#include "stdint.h"
#include "stm32f103xb.h"

void Tim3_Init(void);

void Tim3_Start(void);

void Tim3_Stop(void);

// Задержки modbus
void Tim2_Init(void);

void Tim2_Start(uint16_t arr);

void Tim2_Stop(void);
#endif