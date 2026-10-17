#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

typedef enum { HEALTH_ADC, HEALTH_UART, HEALTH_LED, HEALTH_TASKS } HealthTask;
typedef enum {
    COUNT_RX_BYTES, COUNT_RX_FRAMES, COUNT_TX_FRAMES, COUNT_CRC_ERRORS,
    COUNT_UART_ERRORS, COUNT_RX_OVERFLOW, COUNT_QUEUE_OVERFLOW,
    COUNT_ADC_DMA_ERRORS, COUNT_ADC_RESTARTS, COUNT_TX_DMA_ERRORS,
    COUNT_TX_TIMEOUTS, COUNT_MODBUS_EXCEPTIONS, COUNT_FLASH_FAILURES,
    HEALTH_COUNTERS
} HealthCounter;
typedef enum {
    FAULT_NONE, FAULT_ERROR_HANDLER, FAULT_HARD, FAULT_MEMORY,
    FAULT_BUS, FAULT_USAGE, FAULT_NMI, FAULT_STACK, FAULT_HEAP,
    FAULT_ADC_STALL, FAULT_UART_STALL
} HealthFault;
typedef struct {
    uint32_t reset_flags, uptime_s, uid[3], counters[HEALTH_COUNTERS];
    uint16_t last_fault, free_heap, min_heap, stack[HEALTH_TASKS], alive;
} HealthSnapshot;
void Health_Init(void);
void Health_StartWatchdog(void);
void Health_RegisterTask(HealthTask task, TaskHandle_t handle);
void Health_Heartbeat(HealthTask task);
void Health_Idle(void);
void Health_Increment(HealthCounter counter);
void Health_Snapshot(HealthSnapshot *snapshot);
void Health_Panic(HealthFault fault) __attribute__((noreturn));
