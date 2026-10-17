#pragma once
#include "FreeRTOS.h"
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned depth, void *arg, unsigned priority, TaskHandle_t *handle);
TickType_t xTaskGetTickCount(void);
void vTaskDelay(TickType_t ticks);

void vTaskStartScheduler(void);

unsigned uxTaskGetStackHighWaterMark(TaskHandle_t handle);
