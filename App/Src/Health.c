#include "Health.h"
#include "stm32f1xx.h"
#include <string.h>

static volatile uint32_t counters[HEALTH_COUNTERS];
static TaskHandle_t handles[HEALTH_TASKS];
static volatile TickType_t heartbeat[HEALTH_TASKS];
static uint32_t reset_flags, uptime_s, remainder_ms;
static TickType_t uptime_tick;
static uint16_t previous_fault;
static bool started, fault_recorded;
#define FAULT_MAGIC 0x4821U

static void record_fault(HealthFault fault)
{
    /* Backup registers survive watchdog/software reset while board power remains. */
    BKP->DR1 = (uint16_t)fault;
    BKP->DR2 = FAULT_MAGIC;
}
void Health_Init(void)
{
    reset_flags = RCC->CSR & 0xFE000000UL;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    previous_fault = BKP->DR2 == FAULT_MAGIC ? (uint16_t)BKP->DR1 : FAULT_NONE;
    BKP->DR1 = 0U; BKP->DR2 = 0U;
    RCC->CSR |= RCC_CSR_RMVF;
    memset((void *)counters, 0, sizeof(counters));
    started = false; fault_recorded = false;
    uptime_s = remainder_ms = 0U; uptime_tick = 0U;
}
void Health_RegisterTask(HealthTask task, TaskHandle_t handle) { if (task < HEALTH_TASKS) handles[task] = handle; }
void Health_Heartbeat(HealthTask task) { if (task < HEALTH_TASKS) heartbeat[task] = xTaskGetTickCount(); }
void Health_Increment(HealthCounter counter)
{
    if (counter >= HEALTH_COUNTERS) return;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (counters[counter] != UINT32_MAX) ++counters[counter];
    __set_PRIMASK(mask);
}
void Health_StartWatchdog(void)
{
    /* IWDG: LSI 30..60 kHz, /64, reload 4095 -> approx 4.37..8.74 s.
     * Readiness timeout is 1 s; UART heartbeats also run during long TX. */
    RCC->CSR |= RCC_CSR_LSION;
    uint32_t ready_limit = 1000000U;
    while (!(RCC->CSR & RCC_CSR_LSIRDY) && --ready_limit) { }
    if (!ready_limit) Health_Panic(FAULT_ERROR_HANDLER);
    /* F1 watchdog register updates require its peripheral clock to be running.
     * Start first, as in HAL_IWDG_Init, then unlock/configure/reload. */
    IWDG->KR = 0xCCCCU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 4U;
    IWDG->RLR = 4095U;
    uint32_t limit = 1000000U;
    while (IWDG->SR && --limit) { }
    if (!limit) Health_Panic(FAULT_ERROR_HANDLER);
    IWDG->KR = 0xAAAAU;
    TickType_t now = xTaskGetTickCount();
    for (unsigned i = 0; i < HEALTH_TASKS; ++i) heartbeat[i] = now;
    uptime_tick = now;
    started = true;
}
void Health_Idle(void)
{
    if (!started) return;
    TickType_t now = xTaskGetTickCount();
    remainder_ms += (uint32_t)(now - uptime_tick);
    uptime_tick = now;
    uptime_s += remainder_ms / 1000U;
    remainder_ms %= 1000U;
    bool adc = (TickType_t)(now - heartbeat[HEALTH_ADC]) < pdMS_TO_TICKS(1000U);
    bool uart = (TickType_t)(now - heartbeat[HEALTH_UART]) < pdMS_TO_TICKS(1000U);
    if (adc && uart) {
        IWDG->KR = 0xAAAAU;
        if (fault_recorded) { BKP->DR1 = 0U; BKP->DR2 = 0U; fault_recorded = false; }
    }
    else if (!fault_recorded) {
        record_fault(!adc ? FAULT_ADC_STALL : FAULT_UART_STALL);
        fault_recorded = true;
    }
}
void Health_Snapshot(HealthSnapshot *s)
{
    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL();
    s->reset_flags = reset_flags; s->uptime_s = uptime_s;
    s->last_fault = previous_fault;
    for (unsigned i = 0; i < HEALTH_COUNTERS; ++i) s->counters[i] = counters[i];
    s->alive = 0U;
    for (unsigned i = 0; i < HEALTH_TASKS; ++i)
        if (started && (TickType_t)(now - heartbeat[i]) < pdMS_TO_TICKS(1000U)) s->alive |= 1U << i;
    taskEXIT_CRITICAL();
    const volatile uint32_t *uid = (const volatile uint32_t *)0x1FFFF7E8UL;
    for (unsigned i = 0; i < 3U; ++i) s->uid[i] = uid[i];
    s->free_heap = (uint16_t)xPortGetFreeHeapSize();
    s->min_heap = (uint16_t)xPortGetMinimumEverFreeHeapSize();
    for (unsigned i = 0; i < HEALTH_TASKS; ++i)
        s->stack[i] = handles[i] ? (uint16_t)uxTaskGetStackHighWaterMark(handles[i]) : 0U;
}
void Health_Panic(HealthFault fault)
{
    __disable_irq();
    /* Release the bus even if the failure happened while DMA was transmitting. */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->BRR = GPIO_BRR_BR8;
    GPIOA->CRH = (GPIOA->CRH & ~0xFUL) | 0x2U;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    record_fault(fault);
    __DSB();
    NVIC_SystemReset();
    for (;;) { }
}

void Health_AssertFailed(void) { Health_Panic(FAULT_ERROR_HANDLER); }
