#include "UART.h"
#include "Modbus.h"
#include "ModbusRegisters.h"
#include "stdint.h"
#include "stm32f1xx.h"
#include <math.h>

static volatile uint8_t uart_tx_busy = 0;

uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];

void uart_init(uint32_t baudrate)
{

    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    uart_dma_tx_init();
    uart_dma_rx_init();

    uart_set_baudrate(baudrate);
    uart_cr_init();
    uart_cr_start();

    uart_dma_rx_start();
    // Implementation for UART initialization
}

bool uart_change_setings()
{
    uart_cr_stop();
    uart_set_baudrate(get_baud_from_code(connection_data_active.baudrate_code));
    uart_cr_start();
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

    if (sr & USART_SR_IDLE)
    {
        volatile uint32_t tmp;

        tmp = USART1->SR;
        tmp = USART1->DR;
        (void)tmp;

        DMA1_Channel5->CCR &= ~DMA_CCR_EN;

        uint16_t received_len = UART_RX_BUFFER_SIZE - DMA1_Channel5->CNDTR;

        if ((received_len > 0U) && (received_len <= UART_RX_BUFFER_SIZE))
        {
            uint16_t uart_tx_len = 0U;
            uint8_t result = modbus_read_frame(uart_rx_buffer, received_len, uart_tx_buffer, &uart_tx_len);
            if (result == MB_EX_NONE)
            {
                uart_send(uart_tx_buffer, uart_tx_len);
            }
        }

        uart_dma_rx_start();
    }
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

    USART1->CR1 |= USART_CR1_IDLEIE |
                   USART_CR1_TE |
                   USART_CR1_RE;

    USART1->CR3 |= USART_CR3_DMAR;

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