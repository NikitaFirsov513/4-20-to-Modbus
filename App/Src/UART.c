#include "UART.h"
#include "Modbus.h"
#include "ModbusRegisters.h"
#include "Tim.h"
#include "stdbool.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include <math.h>

static volatile uint8_t uart_tx_busy = 0;

static bool is_frame_start = false;
static bool is_frame_error = false;
static uint16_t t1_us = 0;

uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t rx_index = 0;

void uart_init(uint32_t baudrate, uint16_t parity_bit, uint16_t stop_bit)
{

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    Tim2_Init();
    uart_dma_tx_init();
    // uart_dma_rx_init();
    t1_us = uart_calculate_us(baudrate, parity_bit, stop_bit);
    uart_set_baudrate(baudrate);
    uart_set_parity(parity_bit);
    uart_cr_init();
    uart_cr_start();

    // uart_dma_rx_start();
    //  Implementation for UART initialization
}

bool uart_change_setings()
{
    uart_cr_stop();

    uart_set_baudrate(get_baud_from_code(connection_data_active.baudrate_code));
    uart_set_parity(connection_data_active.parity_code);
    uart_set_stopbits(connection_data_active.stop_bits);
    uart_cr_start();
    return true;
}

uint8_t uart_send(const uint8_t *data, uint16_t len)
{
    if ((data == 0) || (len == 0U) || (len > UART_TX_BUFFER_SIZE))
    {
        return 0U;
    }

    if (uart_tx_busy)
    {
        return 0U;
    }

    uart_tx_busy = 1U;

    for (uint16_t i = 0; i < len; i++)
    {
        uart_tx_buffer[i] = data[i];
    }

    uart_dma_tx_start(len);

    return 1U;
}

void uart_irq_handler(void)
{
    uint32_t sr = USART1->SR;

    if (sr & USART_SR_ORE)
    {
        volatile uint32_t tmp = USART1->DR;
        (void)tmp;

        is_frame_error = true;

        return;
    }

    if (sr & USART_SR_RXNE)
    {
        uint8_t byte = (uint8_t)USART1->DR;

        uint16_t rx_byte_timeout = 5U * t1_us / 2U;
        uint16_t frame_timeout = 9U * t1_us / 2U;

        if (is_frame_start)
        {
            uint16_t counter = TIM2->CNT;

            if (counter > rx_byte_timeout)
            {
                is_frame_error = true;
            }

            if (rx_index < UART_RX_BUFFER_SIZE)
            {
                uart_rx_buffer[rx_index++] = byte;
            }
            else
            {
                is_frame_error = true;
            }

            Tim2_Start(frame_timeout);
        }
        else
        {
            if (rx_index < UART_RX_BUFFER_SIZE)
            {
                uart_rx_buffer[rx_index++] = byte;
            }
            else
            {
                is_frame_error = true;
            }

            is_frame_start = true;

            Tim2_Start(frame_timeout);
        }
    }
}

static void clear_rx_array(void)
{

    for (uint8_t i = 0; i < rx_index; i++)
    {
        uart_rx_buffer[i] = 0U;
    }
    rx_index = 0;
}

void uart_tim2_irq_handler(void)
{
    // тут обработака кадра
    // если больше чем нужно то отбрасываем кадр
    // если кадр норм то читаем и вызваем отбработчик
    // если нет то мигаем светодиодом

    if ((rx_index > 0U) && (rx_index <= UART_RX_BUFFER_SIZE))
    {
        uint16_t uart_tx_len = 0U;
        uint8_t result = modbus_read_frame(uart_rx_buffer, rx_index, uart_tx_buffer, &uart_tx_len);
        if (result == MB_EX_NONE && !is_frame_error)
        {
            uart_send(uart_tx_buffer, uart_tx_len);
        }
        else
        {
            // ошибка
        }
        // rx_index = 0;
        clear_rx_array();
    }
    is_frame_error = false;
    is_frame_start = false;
}

uint16_t uart_calculate_us(uint32_t baudrate,
                           uint16_t parity_bit,
                           uint16_t stop_bit)
{
    uint8_t Nstop;

    if (stop_bit > 1U)
    {
        Nstop = 2U;
    }
    else
    {
        Nstop = 1U;
    }

    uint16_t N = 1U + 8U + parity_bit + Nstop;

    uint16_t t =
        (uint16_t)(((uint32_t)N * 1000000U) / baudrate);

    return t;
}

void uart_dma_tx_irq_handler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF4)
    {
        DMA1->IFCR = DMA_IFCR_CTCIF4;

        DMA1_Channel4->CCR &= ~DMA_CCR_EN;

        USART1->CR3 &= ~USART_CR3_DMAT;

        uart_tx_busy = 0U;
    }

    if (DMA1->ISR & DMA_ISR_TEIF4)
    {
        DMA1->IFCR = DMA_IFCR_CTEIF4;

        DMA1_Channel4->CCR &= ~DMA_CCR_EN;

        USART1->CR3 &= ~USART_CR3_DMAT;

        uart_tx_busy = 0U;
    }
}

void uart_dma_rx_irq_handler(void)
{
    if (DMA1->ISR & DMA_ISR_TCIF5)
    {
        DMA1->IFCR = DMA_IFCR_CTCIF5;

        /*
         * RX DMA заполнил весь буфер.
         * Для твоего теста просто перезапустим RX.
         */
        uart_dma_rx_start();
    }

    if (DMA1->ISR & DMA_ISR_TEIF5)
    {
        DMA1->IFCR = DMA_IFCR_CTEIF5;

        uart_dma_rx_start();
    }
}

static void uart_cr_init(void)
{
    USART1->CR1 &= ~USART_CR1_UE;

    USART1->CR1 = 0;
    USART1->CR2 = 0;
    USART1->CR3 = 0;

    USART1->CR1 |=
        USART_CR1_TE |
        USART_CR1_RE |
        USART_CR1_RXNEIE;

    // USART1->CR3 |= USART_CR3_DMAR;

    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART1_IRQn);
}

static void uart_cr_start(void)
{ // включение UART
    USART1->CR1 |= USART_CR1_UE;
}

static void uart_cr_stop(void)
{ // включение UART
    USART1->CR1 &= ~USART_CR1_UE;
}

static void uart_set_baudrate(uint32_t baudrate)
{
    if (baudrate == 0U)
    {
        return;
    }

    uint32_t brr = (UART_PCLK_HZ + (baudrate / 2U)) / baudrate;

    USART1->BRR = (uint16_t)brr;
}

static void uart_set_parity(uint16_t parity)
{
    switch (parity)
    {
    case MB_PARITY_NONE:
        USART1->CR1 &= ~USART_CR1_PCE;
        break;

    case MB_PARITY_EVEN:
        USART1->CR1 |= USART_CR1_PCE;
        USART1->CR1 &= ~USART_CR1_PS;
        USART1->CR1 |= USART_CR1_M;
        break;

    case MB_PARITY_ODD:
        USART1->CR1 |= USART_CR1_PCE;
        USART1->CR1 |= USART_CR1_PS;
        USART1->CR1 |= USART_CR1_M;
        break;

    default:
        USART1->CR1 &= ~USART_CR1_PCE;
        break;
    }
}

static void uart_set_stopbits(uint16_t stop_bits)
{

    // 00 → 1 stop bit 01 → 0.5 stop bit 10 → 2 stop bits 11 → 1.5 stop bit
    switch (stop_bits)
    {
    case 0U:
        /* code */
        USART1->CR2 &= ~USART_CR2_STOP_1;
        USART1->CR2 &= ~USART_CR2_STOP;
        break;
    case 1U:
        /* code */
        USART1->CR2 &= ~USART_CR2_STOP_1;
        USART1->CR2 |= USART_CR2_STOP;
        break;
    case 2U:
        /* code */
        USART1->CR2 |= USART_CR2_STOP_1;
        USART1->CR2 &= ~USART_CR2_STOP;
        break;
    case 3U:
        /* code */
        USART1->CR2 |= USART_CR2_STOP_1;
        USART1->CR2 |= USART_CR2_STOP;
        break;
    default:
        break;
    }
}

static void uart_dma_init(void)
{

    uart_dma_tx_init();
    uart_dma_rx_init();
    uart_dma_rx_start();
}

static void uart_dma_tx_init(void)
{
    // TX
    DMA1->IFCR = DMA_IFCR_CGIF4 |
                 DMA_IFCR_CTCIF4 |
                 DMA_IFCR_CHTIF4 |
                 DMA_IFCR_CTEIF4;
    DMA1_Channel4->CCR = 0;
    DMA1_Channel4->CCR |= DMA_CCR_TCIE;
    DMA1_Channel4->CCR |= DMA_CCR_TEIE;
    NVIC_SetPriority(DMA1_Channel4_IRQn, 5);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    DMA1_Channel4->CCR |= DMA_CCR_DIR;
    DMA1_Channel4->CCR |= DMA_CCR_MINC;
    DMA1_Channel4->CCR |= DMA_CCR_PL_1;
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel4->CMAR = (uint32_t)uart_tx_buffer;
}

static void uart_dma_tx_start(uint16_t len)
{

    if ((len == 0U) || (len > UART_TX_BUFFER_SIZE))
    {
        return;
    }

    DMA1_Channel4->CCR &= ~DMA_CCR_EN;

    DMA1->IFCR = DMA_IFCR_CGIF4 |
                 DMA_IFCR_CTCIF4 |
                 DMA_IFCR_CHTIF4 |
                 DMA_IFCR_CTEIF4;

    DMA1_Channel4->CMAR = (uint32_t)uart_tx_buffer;
    DMA1_Channel4->CNDTR = len;

    USART1->CR3 |= USART_CR3_DMAT;

    DMA1_Channel4->CCR |= DMA_CCR_EN;
}

static void uart_dma_rx_init(void)
{
    // RX
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    DMA1->IFCR = DMA_IFCR_CGIF5 |
                 DMA_IFCR_CTCIF5 |
                 DMA_IFCR_CHTIF5 |
                 DMA_IFCR_CTEIF5;

    DMA1_Channel5->CCR = 0;

    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR;
    DMA1_Channel5->CMAR = (uint32_t)uart_rx_buffer;
    DMA1_Channel5->CNDTR = UART_RX_BUFFER_SIZE;

    DMA1_Channel5->CCR =
        DMA_CCR_TCIE |
        DMA_CCR_TEIE |
        DMA_CCR_MINC |
        DMA_CCR_PL_1;

    NVIC_SetPriority(DMA1_Channel5_IRQn, 5);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

static void uart_dma_rx_start(void)
{
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    DMA1->IFCR = DMA_IFCR_CGIF5 |
                 DMA_IFCR_CTCIF5 |
                 DMA_IFCR_CHTIF5 |
                 DMA_IFCR_CTEIF5;

    DMA1_Channel5->CMAR = (uint32_t)uart_rx_buffer;
    DMA1_Channel5->CNDTR = UART_RX_BUFFER_SIZE;

    DMA1_Channel5->CCR |= DMA_CCR_EN;
}