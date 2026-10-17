#include "UART.h"
#include "Health.h"
#include "Modbus.h"
#include "ModbusRegisters.h"
#include "Tim.h"
#include "LedTask.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "stm32f1xx.h"
#include <string.h>

typedef struct { uint16_t len; uint8_t data[UART_RX_BUFFER_SIZE]; } RxFrame;
static RxFrame rx;
static QueueHandle_t frames;
static uint8_t tx_buffer[UART_TX_BUFFER_SIZE];
static volatile bool tx_busy;
static volatile bool tx_failed;
static bool rx_error;
static uint16_t byte_timeout_us, frame_timeout_us;
static uint32_t character_us;

static void receive_mode(void)
{
    USART1->CR1 &= ~USART_CR1_TCIE;
    GPIOA->BRR = GPIO_BRR_BR8; /* PA8: tied DE and /RE, low = receive. */
    volatile uint32_t discard = USART1->SR;
    discard = USART1->DR;
    (void)discard;
    USART1->CR1 |= USART_CR1_RE;
    if (tx_busy && !tx_failed) Health_Increment(COUNT_TX_FRAMES);
    tx_busy = false;
}

static void abort_tx(void)
{
    taskENTER_CRITICAL();
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    USART1->CR3 &= ~USART_CR3_DMAT;
    USART1->CR1 &= ~USART_CR1_TE;
    receive_mode();
    USART1->CR1 |= USART_CR1_TE;
    taskEXIT_CRITICAL();
}

static void UartTask(void *argument)
{
    (void)argument;
    RxFrame frame;
    uint8_t response[MODBUS_MAX_ADU];
    for (;;) {
        Health_Heartbeat(HEALTH_UART);
        if (xQueueReceive(frames, &frame, pdMS_TO_TICKS(100U)) != pdPASS) continue;
        uint16_t len = 0U;
        uint8_t result = modbus_read_frame(frame.data, frame.len, response, &len);
        bool completed = true;
        if (len && uart_send(response, len)) {
            TickType_t started = xTaskGetTickCount();
            TickType_t timeout = pdMS_TO_TICKS((character_us * len + 999U) / 1000U + 50U);
            while (uart_is_busy()) {
                Health_Heartbeat(HEALTH_UART);
                if ((TickType_t)(xTaskGetTickCount() - started) > timeout) {
                    Health_Increment(COUNT_TX_TIMEOUTS);
                    abort_tx();
                    tx_failed = true;
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            completed = !tx_failed;
        } else if (len) completed = false;
        /* TC has released DE; acknowledgements use the old line parameters. */
        if (completed) modbus_apply_pending_config();
        else modbus_cancel_pending_config();
        LedTask_SendMessage(!completed ? LED_ERROR_SPEED : result == MB_EX_NONE ? LED_MESSAGE_OK : LED_ERROR_FORMAT);
    }
}

bool uart_task_start(void)
{
    frames = xQueueCreate(2U, sizeof(RxFrame));
    if (!frames) return false;
    TaskHandle_t handle;
    if (xTaskCreate(UartTask,"modbus",640U,NULL,3U,&handle) != pdPASS) return false;
    Health_RegisterTask(HEALTH_UART,handle);
    return true;
}

uint16_t uart_calculate_us(uint32_t baudrate, uint16_t parity, uint16_t stop_bits)
{
    if (!baudrate) return 0U;
    uint32_t bits = 1U + 8U + (parity != MB_PARITY_NONE ? 1U : 0U) + stop_bits;
    return (uint16_t)((bits * 1000000UL + baudrate - 1U) / baudrate);
}

void uart_init(uint32_t baudrate, uint16_t parity, uint16_t stop_bits)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    Tim2_Init();
    USART1->CR1 = 0U;
    USART1->CR2 = stop_bits == 2U ? USART_CR2_STOP_1 : 0U;
    USART1->CR3 = USART_CR3_EIE;
    USART1->BRR = (UART_PCLK_HZ + baudrate / 2U) / baudrate;
    uint32_t cr1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    if (parity != MB_PARITY_NONE) {
        cr1 |= USART_CR1_M | USART_CR1_PCE | USART_CR1_PEIE;
        if (parity == MB_PARITY_ODD) cr1 |= USART_CR1_PS;
    }
    character_us = uart_calculate_us(baudrate, parity, stop_bits);
    /* RXNE occurs at the end of a character. Compare end-to-end time:
     * maximum idle gap t1.5 plus the duration of the next character. */
    byte_timeout_us = baudrate > 19200U ? 750U + character_us : (5U * character_us + 1U) / 2U;
    frame_timeout_us = baudrate > 19200U ? 1750U : (7U * character_us + 1U) / 2U;
    rx.len = 0U; rx_error = false; tx_busy = false;
    GPIOA->BRR = GPIO_BRR_BR8;
    DMA1_Channel4->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_PL_1;
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel4->CMAR = (uint32_t)tx_buffer;
    DMA1->IFCR = DMA_IFCR_CGIF4;
    NVIC_SetPriority(DMA1_Channel4_IRQn, 5U);
    NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    NVIC_SetPriority(USART1_IRQn, 5U);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
    USART1->CR1 = cr1 | USART_CR1_UE;
}

bool uart_change_setings(void)
{
    if (tx_busy || !modbus_config_valid(&connection_data_active)) return false;
    taskENTER_CRITICAL();
    Tim2_Stop();
    uart_init(get_baud_from_code(connection_data_active.baudrate_code),
              connection_data_active.parity_code, connection_data_active.stop_bits);
    xQueueReset(frames); /* Queued requests belong to the old configuration. */
    taskEXIT_CRITICAL();
    return true;
}

bool uart_is_busy(void) { return tx_busy; }

uint8_t uart_send(const uint8_t *data, uint16_t len)
{
    if (!data || !len || len > sizeof(tx_buffer)) return 0U;
    taskENTER_CRITICAL();
    if (tx_busy) { taskEXIT_CRITICAL(); return 0U; }
    tx_busy = true;
    tx_failed = false;
    memcpy(tx_buffer, data, len);
    Tim2_Stop();
    rx.len = 0U; rx_error = false;
    USART1->CR1 &= ~USART_CR1_RE;
    GPIOA->BSRR = GPIO_BSRR_BS8; /* Drive DE high before the first start bit. */
    __DSB();
    /* CA-IS2092VW tPZH/tPZL max 800 ns. At 64 MHz these 128 NOPs
     * provide at least 2 us, independently of loop/Flash wait-state overhead. */
    for (uint32_t i = 0U; i < 128U; ++i) __NOP();
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1->IFCR = DMA_IFCR_CGIF4;
    DMA1_Channel4->CNDTR = len;
    USART1->SR = ~USART_SR_TC;
    USART1->CR3 |= USART_CR3_DMAT;
    DMA1_Channel4->CCR |= DMA_CCR_EN;
    taskEXIT_CRITICAL();
    return 1U;
}

void uart_irq_handler(void)
{
    uint32_t sr = USART1->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        uint8_t byte = (uint8_t)USART1->DR; /* SR then DR clears receive errors. */
        if (!tx_busy) {
            if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
                rx_error = true; Health_Increment(COUNT_UART_ERRORS);
            }
            if (rx.len && TIM2->CNT > byte_timeout_us) { rx_error = true; Health_Increment(COUNT_UART_ERRORS); }
            if (sr & USART_SR_RXNE) {
                Health_Increment(COUNT_RX_BYTES);
                if (rx.len < sizeof(rx.data)) rx.data[rx.len++] = byte;
                else { rx_error = true; Health_Increment(COUNT_RX_OVERFLOW); }
            }
            Tim2_Start(frame_timeout_us);
        }
    }
    if ((sr & USART_SR_TC) && (USART1->CR1 & USART_CR1_TCIE)) receive_mode();
}

void uart_tim2_irq_handler(void)
{
    BaseType_t wake = pdFALSE;
    if (rx.len && !rx_error && !tx_busy && frames) {
        Health_Increment(COUNT_RX_FRAMES);
        if (xQueueSendFromISR(frames, &rx, &wake) != pdPASS) {
            Health_Increment(COUNT_QUEUE_OVERFLOW);
            LedTask_SendMessageFromISR(LED_ERROR_FORMAT);
        }
    } else if (rx_error) LedTask_SendMessageFromISR(LED_ERROR_FORMAT);
    rx.len = 0U; rx_error = false;
    portYIELD_FROM_ISR(wake);
}

void uart_dma_tx_irq_handler(void)
{
    uint32_t flags = DMA1->ISR;
    if (flags & (DMA_ISR_TCIF4 | DMA_ISR_TEIF4)) {
        if (flags & DMA_ISR_TEIF4) { tx_failed = true; Health_Increment(COUNT_TX_DMA_ERRORS); }
        DMA1->IFCR = DMA_IFCR_CGIF4;
        DMA1_Channel4->CCR &= ~DMA_CCR_EN;
        USART1->CR3 &= ~USART_CR3_DMAT;
        /* DMA TC means DR is filled, not that the stop bit has left TX.
         * Also drain the serializer after DMA error before dropping DE. */
        USART1->CR1 |= USART_CR1_TCIE;
    }
}
