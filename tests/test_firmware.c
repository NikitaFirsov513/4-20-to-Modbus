#include "Modbus.h"
#include "Health.h"
#include "ModbusRegisters.h"
#include "SensorMath.h"
#include "UART.h"
#include "Tim.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "stm32f1xx.h"
#include <stddef.h>
#include <string.h>

#define CHECK(condition) do { ++checks; if (!(condition)) return __LINE__; } while (0)
unsigned checks;
static FlashConfigT saved;
static bool have_saved, flash_failure;
static unsigned flash_writes;
static uint8_t queued[258];
static unsigned queue_size, queued_count;
static TickType_t ticks;
static bool service_mode;
bool App_DefaultsForced(void) { return service_mode; }
uint16_t App_StorageFlags(void) { return have_saved ? 11U : 0U; }
size_t xPortGetFreeHeapSize(void) { return 5000; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return 4000; }
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t handle) { (void)handle; return 128; }

bool App_LoadConfig(FlashConfigT *config) { *config = saved; return have_saved; }
bool App_SaveConfig(const FlashConfigT *config) {
    ++flash_writes;
    if (flash_failure) return false;
    saved = *config; have_saved = true; return true;
}
void LedTask_SendMessage(uint8_t m) { (void)m; }
void LedTask_SendMessageFromISR(uint8_t m) { (void)m; }
QueueHandle_t xQueueCreate(unsigned count, unsigned size) { (void)count; queue_size = size; return queued; }
BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t timeout) { (void)q; (void)item; (void)timeout; return 0; }
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *item, BaseType_t *wake) {
    (void)q; *wake = 0; memcpy(queued, item, queue_size); ++queued_count; return pdPASS;
}
BaseType_t xQueueReset(QueueHandle_t q) { (void)q; queued_count = 0; return pdPASS; }
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, unsigned depth, void *arg, unsigned priority, TaskHandle_t *handle) {
    (void)fn; (void)name; (void)depth; (void)arg; (void)priority; (void)handle; return pdPASS;
}
TickType_t xTaskGetTickCount(void) { return ticks; }
void vTaskDelay(TickType_t t) { ticks += t; }

static uint16_t finish(uint8_t *frame, uint16_t payload) {
    uint16_t crc = modbus_crc16(frame, payload);
    frame[payload] = (uint8_t)crc; frame[payload + 1] = crc >> 8;
    return payload + 2;
}
static uint8_t run(uint8_t *frame, uint16_t n, uint8_t *tx, uint16_t *len) {
    return modbus_read_frame(frame, finish(frame, n), tx, len);
}
static int protocol_tests(void)
{
    uint8_t frame[256] = {1,3,0,0,0,10}, tx[256];
    uint16_t len = 123U;
    CHECK(modbus_crc16(frame,6) == 0xCDC5U);
    modbus_init();
    CHECK(connection_data_active.slave_address == 1 && connection_data.baudrate_code == MB_BAUD_9600);
    CHECK(modbus_read_frame(NULL, 0, tx, &len) == MB_EX_ILLEGAL_DATA_VALUE && len == 0);
    CHECK(!modbus_verify_crc(NULL, 8));
    CHECK(modbus_read_frame(frame, 8, tx, NULL) == MB_EX_SLAVE_DEVICE_FAILURE);
    frame[1] = 6;
    CHECK(run(frame, 2, tx, &len) == MB_EX_ILLEGAL_DATA_VALUE && len == 5 && tx[1] == 0x86);
    CHECK(flash_writes == 0 && connection_data.slave_address == 1);
    frame[1] = 16; frame[2] = 0; frame[3] = 1; frame[4] = 0; frame[5] = 4; frame[6] = 8;
    CHECK(run(frame, 7, tx, &len) == MB_EX_ILLEGAL_DATA_VALUE && len == 5);
    CHECK(connection_data.slave_address == 1);
    frame[1] = 3; frame[2] = 1; frame[3] = 0; frame[4] = 0; frame[5] = 0;
    CHECK(run(frame, 6, tx, &len) == MB_EX_ILLEGAL_DATA_VALUE && tx[2] == 3);
    frame[5] = 126;
    CHECK(run(frame, 6, tx, &len) == MB_EX_ILLEGAL_DATA_VALUE);
    frame[5] = 4;
    ADC_data_t adc = {12000, 1700, 2100, 0x02};
    modbus_publish_adc(&adc);
    CHECK(run(frame, 6, tx, &len) == MB_EX_NONE && len == 13 && modbus_verify_crc(tx,len));
    CHECK(tx[3] == (12000 >> 8) && tx[4] == (12000 & 255) && tx[10] == 0x02);
    connection_data_active.slave_address = 247; frame[0] = 247;
    CHECK(run(frame, 6, tx, &len) == 0 && tx[0] == 247);
    frame[0] = 126; connection_data_active.slave_address = 126;
    CHECK(run(frame, 6, tx, &len) == 0 && tx[0] == 126);
    frame[0] = 1; connection_data_active.slave_address = 1;
    frame[1] = 0x7f;
    CHECK(run(frame, 2, tx, &len) == MB_EX_ILLEGAL_FUNCTION && tx[1] == 0xff && len == 5);
    frame[0] = 2;
    CHECK(run(frame, 2, tx, &len) == MB_EX_ILLEGAL_DATA_ADDRESS && len == 0);
    frame[0] = 1; frame[1] = 6; frame[2] = 1; frame[3] = 0; frame[4] = 0; frame[5] = 10;
    CHECK(run(frame, 6, tx, &len) == MB_EX_ILLEGAL_DATA_ADDRESS && tx[2] == 2);
    frame[2] = 0; frame[3] = 1; frame[4] = 0; frame[5] = 0;
    CHECK(run(frame, 6, tx, &len) == MB_EX_ILLEGAL_DATA_VALUE && connection_data.slave_address == 1);
    frame[5] = 248;
    CHECK(run(frame, 6, tx, &len) == MB_EX_ILLEGAL_DATA_VALUE);
    frame[0] = 0; frame[5] = 42;
    CHECK(run(frame, 6, tx, &len) == 0 && len == 0 && connection_data.slave_address == 42);
    frame[1] = 3;
    CHECK(run(frame, 6, tx, &len) == 0 && len == 0);
    frame[0] = 1; frame[1] = 6; frame[5] = 43;
    uint16_t n = finish(frame, 6); frame[n-1] ^= 1;
    CHECK(modbus_read_frame(frame,n,tx,&len) != 0 && len == 0 && connection_data.slave_address == 42);
    /* A complete 0x10 request is validated before committing any register. */
    uint8_t multiple[13] = {1,16,0,1,0,2,4,0,44,0,7};
    CHECK(run(multiple,11,tx,&len) == 0 && len == 8 && connection_data.slave_address == 44 && connection_data.baudrate_code == 7);
    multiple[8] = 45; multiple[10] = 8;
    CHECK(run(multiple,11,tx,&len) == MB_EX_ILLEGAL_DATA_VALUE && connection_data.slave_address == 44);
    multiple[6] = 2;
    CHECK(run(multiple,11,tx,&len) == MB_EX_ILLEGAL_DATA_VALUE);
    frame[1] = 3; frame[2] = 255; frame[3] = 255; frame[4] = 0; frame[5] = 2;
    CHECK(run(frame,6,tx,&len) == MB_EX_ILLEGAL_DATA_ADDRESS);
    return 0;
}
static int settings_tests(void)
{
    have_saved = false; modbus_init();
    uint8_t data[] = {0,1, 0,247, 0,7, 0,2, 0,2};
    CHECK(modbus_registers_write_many(0,5,data) == 0);
    CHECK(connection_data.slave_address == 247 && connection_data_active.slave_address == 1);
    CHECK(flash_writes == 1 && modbus_verify_crc((uint8_t *)&saved,sizeof(saved)));
    modbus_apply_pending_config();
    CHECK(connection_data_active.slave_address == 247);
    CHECK(USART1->BRR == 556 && (USART1->CR1 & USART_CR1_PS));
    CHECK((USART1->CR2 & USART_CR2_STOP) == USART_CR2_STOP_1);
    modbus_init();
    CHECK(connection_data_active.slave_address == 247 && connection_data.baudrate_code == 7);
    unsigned writes = flash_writes;
    data[3] = 12; data[5] = 8;
    CHECK(modbus_registers_write_many(0,5,data) == MB_EX_ILLEGAL_DATA_VALUE);
    CHECK(connection_data.slave_address == 247 && flash_writes == writes);
    data[5] = 3; flash_failure = true;
    CHECK(modbus_registers_write_many(0,5,data) == MB_EX_SLAVE_DEVICE_FAILURE);
    CHECK(connection_data.slave_address == 247);
    flash_failure = false;
    CHECK(modbus_registers_write_one(0,0) == MB_EX_ILLEGAL_DATA_VALUE);
    CHECK(modbus_registers_write_one(3,3) == MB_EX_ILLEGAL_DATA_VALUE);
    CHECK(modbus_registers_write_one(4,0) == MB_EX_ILLEGAL_DATA_VALUE);
    CHECK(modbus_registers_write_one(4,3) == MB_EX_ILLEGAL_DATA_VALUE);
    CHECK(modbus_registers_write_one(2,8) == MB_EX_ILLEGAL_DATA_VALUE);
    CHECK(modbus_registers_write_one(5,1) == MB_EX_ILLEGAL_DATA_ADDRESS);
    CHECK(modbus_registers_write_one(1,21) == 0 && modbus_registers_write_one(0,1) == 0);
    modbus_cancel_pending_config();
    modbus_apply_pending_config();
    CHECK(connection_data_active.slave_address == 247);
    saved.slave_address = 0; modbus_init();
    CHECK(connection_data_active.slave_address == 1 && connection_data.baudrate_code == 3);
    return 0;
}
static void rx_byte(uint8_t byte, uint32_t interval, uint32_t errors)
{
    TIM2->CNT = interval; USART1->DR = byte;
    USART1->SR = USART_SR_RXNE | errors;
    uart_irq_handler();
}
static int uart_tests(void)
{
    uart_init(9600, MB_PARITY_ODD, 2);
    CHECK((USART1->CR1 & (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS)) == (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS));
    CHECK((USART1->CR2 & USART_CR2_STOP) == USART_CR2_STOP_1);
    uart_init(9600, MB_PARITY_NONE, 1);
    CHECK(!(USART1->CR1 & (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS)));
    CHECK((USART1->CR2 & USART_CR2_STOP) == 0);
    CHECK(uart_calculate_us(9600,MB_PARITY_EVEN,1) == uart_calculate_us(9600,MB_PARITY_ODD,1));
    CHECK(uart_calculate_us(9600,MB_PARITY_ODD,1) == 1146);
    CHECK(uart_calculate_us(0,0,1) == 0);
    uint8_t bytes[] = {1,2,3};
    CHECK(uart_send(bytes,3) == 1 && uart_is_busy());
    CHECK(GPIOA->BSRR == GPIO_BSRR_BS8 && !(USART1->CR1 & USART_CR1_RE));
    CHECK(DMA1_Channel4->CNDTR == 3 && (USART1->CR3 & USART_CR3_DMAT));
    CHECK(memcmp((const void *)DMA1_Channel4->CMAR,bytes,3) == 0);
    uint8_t other[] = {9,9,9};
    CHECK(uart_send(other,3) == 0 && memcmp((const void *)DMA1_Channel4->CMAR,bytes,3) == 0);
    DMA1->ISR = DMA_ISR_TCIF4; uart_dma_tx_irq_handler();
    CHECK(uart_is_busy() && (USART1->CR1 & USART_CR1_TCIE));
    USART1->SR = USART_SR_TC; uart_irq_handler();
    CHECK(!uart_is_busy() && GPIOA->BRR == GPIO_BRR_BR8 && (USART1->CR1 & USART_CR1_RE));
    CHECK(!(USART1->CR1 & USART_CR1_TCIE));
    CHECK(uart_send(bytes,3) == 1);
    DMA1->ISR = DMA_ISR_TEIF4; uart_dma_tx_irq_handler();
    CHECK(uart_is_busy());
    USART1->SR = USART_SR_TC; uart_irq_handler(); CHECK(!uart_is_busy());
    queued_count = 0;
    for (unsigned i=0;i<8;++i) rx_byte(i,100,0);
    uart_tim2_irq_handler();
    CHECK(queued_count == 1 && queued[0] == 8 && queued[1] == 0);
    queued_count = 0;
    rx_byte(1,0,0); rx_byte(2,3000,0); uart_tim2_irq_handler();
    CHECK(queued_count == 0);
    rx_byte(1,0,0); rx_byte(2,100,USART_SR_ORE); uart_tim2_irq_handler();
    CHECK(queued_count == 0);
    rx_byte(1,0,0); rx_byte(2,100,USART_SR_PE); uart_tim2_irq_handler();
    CHECK(queued_count == 0);
    for (unsigned i=0;i<256;++i) rx_byte(i,100,0);
    uart_tim2_irq_handler();
    CHECK(queued_count == 1 && queued[0] == 0 && queued[1] == 1);
    queued_count = 0;
    for (unsigned i=0;i<257;++i) rx_byte(i,100,0);
    uart_tim2_irq_handler(); CHECK(queued_count == 0);
    uart_init(115200,0,1);
    rx_byte(1,0,0); rx_byte(2,900,0); uart_tim2_irq_handler(); CHECK(queued_count == 0);
    CHECK(TIM2->ARR == 1749);
    uart_init(1200,0,1); rx_byte(1,0,0); CHECK(TIM2->ARR > 29000 && TIM2->ARR < 30000);
    uart_tim2_irq_handler();
    /* A save remains pending while the serializer is busy. */
    CHECK(modbus_registers_write_one(1,33) == 0 && modbus_registers_write_one(0,1) == 0);
    uint16_t old = connection_data_active.slave_address;
    CHECK(uart_send(bytes,3) == 1); modbus_apply_pending_config();
    CHECK(connection_data_active.slave_address == old);
    DMA1->ISR = DMA_ISR_TCIF4; uart_dma_tx_irq_handler();
    USART1->SR = USART_SR_TC; uart_irq_handler(); modbus_apply_pending_config();
    CHECK(connection_data_active.slave_address == 33);
    return 0;
}
static int sensor_tests(void)
{
    ADC_data_t adc;
    /* Actual board samples: 4 mA (233/234), independent check at 10 mA
     * (1614/1615), and 20 mA (3922/3923). Allow half an ADC count plus
     * measured residual nonlinearity at the independent middle point. */
    const unsigned currents[] = {4000,4000,10000,10000,20000,20000};
    const uint16_t raw[] = {233,234,1614,1615,3922,3923};
    for (unsigned i=0;i<6;++i) {
        Sensor_Convert(raw[i],false,&adc);
        CHECK(adc.current_uA > currents[i]-15 && adc.current_uA < currents[i]+15);
        CHECK(adc.status == 0);
    }
    Sensor_Convert(0,true,&adc); CHECK((adc.status & 3U) == 3U);
    Sensor_Convert(4095,false,&adc); CHECK(adc.status & ADC_STATUS_SATURATED);
    CHECK(adc.status & ADC_STATUS_OVERRANGE);
    CHECK(adc.adc_mv == 3300);
    Sensor_Convert(0,false,&adc); CHECK(adc.status & ADC_STATUS_UNDERRANGE);
    Sensor_Convert(65535,false,&adc); CHECK(adc.current_uA == 65535U);
    return 0;
}
static int reliability_tests(void)
{
    have_saved = false; service_mode = false; modbus_init(); ticks = 100U;
    ADC_data_t adc = {10000,1301,1616,0};
    modbus_publish_adc(&adc);
    CHECK(modbus_adc_age_ms() == 0U);
    ticks += 301U; modbus_snapshot_adc(&adc);
    CHECK(adc.status & ADC_STATUS_STALE);
    CHECK(modbus_adc_age_ms() == 301U);
    /* Publishing an explicitly stale value must not refresh its age. */
    modbus_publish_adc(&adc); ticks += 100U;
    CHECK(modbus_adc_age_ms() == 401U);
    adc.status = 0U; modbus_publish_adc(&adc);
    modbus_snapshot_adc(&adc); CHECK(!(adc.status & ADC_STATUS_STALE));
    ticks = UINT32_MAX-100U; modbus_publish_adc(&adc);
    ticks = 50U; CHECK(modbus_adc_age_ms() == 151U); /* Tick wrap. */

    CHECK(modbus_registers_write_one(1,77U) == 0 && modbus_registers_write_one(0,1) == 0);
    modbus_apply_pending_config();
    CHECK(connection_data_active.slave_address == 77U);
    service_mode = true; modbus_init();
    CHECK(connection_data_active.slave_address == 1U && connection_data_active.baudrate_code == 3U);
    CHECK(modbus_registers_write_one(1,88U) == 0 && modbus_registers_write_one(0,1) == 0);
    modbus_apply_pending_config();
    CHECK(connection_data_active.slave_address == 1U && saved.slave_address == 88U);
    uint16_t active;
    CHECK(modbus_registers_read_one(0x0010,&active) == 0 && active == 1U);
    service_mode = false; modbus_init(); CHECK(connection_data_active.slave_address == 88U);
    uint16_t diag[24];
    CHECK(modbus_registers_read_block(0x0200,24,diag) == 0);
    CHECK(diag[0] == 1U && diag[1] == 1U && diag[12] == 4000U && diag[13] == 5000U);
    CHECK(modbus_registers_read_one(0x0030,&active) == 0 && active == 2331U);
    CHECK(modbus_registers_read_one(0x0031,&active) == 0 && active == 39225U);

    SensorStatusFilter filter = {0};
    adc = (ADC_data_t){4000,0,0,ADC_STATUS_LOOP_OPEN};
    Sensor_FilterStatus(&filter,&adc); CHECK(!(adc.status & ADC_STATUS_LOOP_OPEN));
    adc.status = ADC_STATUS_LOOP_OPEN; Sensor_FilterStatus(&filter,&adc);
    CHECK(!(adc.status & ADC_STATUS_LOOP_OPEN));
    adc.status = ADC_STATUS_LOOP_OPEN; Sensor_FilterStatus(&filter,&adc);
    CHECK(adc.status & ADC_STATUS_LOOP_OPEN);
    for (unsigned i=0;i<3U;++i) { adc.status=0U; Sensor_FilterStatus(&filter,&adc); }
    CHECK(!(adc.status & ADC_STATUS_LOOP_OPEN));
    adc.current_uA=3799; Sensor_FilterStatus(&filter,&adc); CHECK(adc.status & ADC_STATUS_UNDERRANGE);
    adc.current_uA=3850; Sensor_FilterStatus(&filter,&adc); CHECK(adc.status & ADC_STATUS_UNDERRANGE);
    adc.current_uA=3901; Sensor_FilterStatus(&filter,&adc); CHECK(!(adc.status & ADC_STATUS_UNDERRANGE));
    adc.current_uA=20501; Sensor_FilterStatus(&filter,&adc); CHECK(adc.status & ADC_STATUS_OVERRANGE);
    adc.current_uA=20450; Sensor_FilterStatus(&filter,&adc); CHECK(adc.status & ADC_STATUS_OVERRANGE);
    adc.current_uA=20399; Sensor_FilterStatus(&filter,&adc); CHECK(!(adc.status & ADC_STATUS_OVERRANGE));
    CHECK(!Sensor_SetCalibration(4000,2000) && !Sensor_SetCalibration(-1,3000));
    CHECK(!Sensor_SetCalibration(2331,40951));

    /* Watchdog refresh depends on both task heartbeats, not Modbus traffic. */
    RCC->CSR |= RCC_CSR_LSIRDY;
    Health_StartWatchdog();
    Health_Heartbeat(HEALTH_ADC); Health_Heartbeat(HEALTH_UART); Health_Idle();
    CHECK(IWDG->KR == 0xAAAAU);
    ticks += 1100U; IWDG->KR = 0x1234U;
    Health_Heartbeat(HEALTH_UART); Health_Idle(); CHECK(IWDG->KR == 0x1234U);
    Health_Heartbeat(HEALTH_ADC); Health_Idle(); CHECK(IWDG->KR == 0xAAAAU);
    ticks += 1100U; IWDG->KR=0x1234U;
    Health_Heartbeat(HEALTH_ADC); Health_Idle(); CHECK(IWDG->KR == 0x1234U);
    Health_Heartbeat(HEALTH_UART); Health_Idle(); CHECK(IWDG->KR == 0xAAAAU);
    Health_Increment(COUNT_CRC_ERRORS);
    HealthSnapshot health; Health_Snapshot(&health);
    CHECK((health.alive & 3U) == 3U && health.counters[COUNT_CRC_ERRORS] > 0U);
    return 0;
}
static uint32_t random_state = 0x81735291U;
static uint32_t rng(void) { random_state ^= random_state << 13; random_state ^= random_state >> 17; return random_state ^= random_state << 5; }
static int fuzz_tests(void)
{
    struct { uint8_t before[16], data[256], after[16]; } tx;
    uint8_t frame[256];
    have_saved = false; modbus_init();
    for (unsigned iteration=0;iteration<20000;++iteration) {
        for (unsigned i=0;i<256;++i) frame[i] = (uint8_t)rng();
        uint16_t length = rng() % 257U;
        if (length >= 4U && (iteration & 1U)) {
            frame[0] = 1U;
            const uint8_t functions[] = {3,6,16,0x7f};
            frame[1] = functions[(iteration >> 1) % 4U];
            finish(frame,length-2U);
        }
        memset(&tx,0xa5,sizeof(tx));
        uint16_t reply = 999U;
        modbus_read_frame(frame,length,tx.data,&reply);
        CHECK(reply <= 256U);
        CHECK(!reply || modbus_verify_crc(tx.data,reply));
        for (unsigned i=0;i<16;++i) CHECK(tx.before[i] == 0xa5 && tx.after[i] == 0xa5);
    }
    return 0;
}
int test_main(void)
{
    Health_Init();
    if (!uart_task_start()) return 999;
    int result;
    if ((result=protocol_tests())) return result;
    if ((result=settings_tests())) return result;
    if ((result=uart_tests())) return result;
    if ((result=sensor_tests())) return result;
    if ((result=reliability_tests())) return result;
    if ((result=fuzz_tests())) return result;
    return 0;
}

int health_reboot_check(void)
{
    Health_Init();
    HealthSnapshot state;
    Health_Snapshot(&state);
    return state.last_fault==FAULT_HARD && (state.reset_flags & RCC_CSR_SFTRSTF) ? 0 : 1;
}
