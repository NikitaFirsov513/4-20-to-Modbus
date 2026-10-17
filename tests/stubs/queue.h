#pragma once
#include "FreeRTOS.h"
typedef void * QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned count, unsigned size);
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t timeout);
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *wake);
BaseType_t xQueueReset(QueueHandle_t q);
