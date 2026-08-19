#pragma once
#include "stdint.h"
#include "stdbool.h"

#define MB_READ_HOLDING_REGISTERS   0x03U
#define MB_WRITE_SINGLE_REGISTER    0x06U
#define MB_WRITE_MULTIPLE_REGISTERS 0x10U

#define UART_TX_BUFFER_SIZE         128
#define UART_RX_BUFFER_SIZE         128
#define UART_PCLK_HZ                64000000UL

void uart_init(uint32_t baudrate);
uint8_t uart_send(const uint8_t *data, uint16_t len);
void uart_irq_handler(void);

void uart_dma_tx_irq_handler(void);
void uart_dma_rx_irq_handler(void);

bool uart_change_setings();

static void uart_set_baudrate(uint32_t baudrate);
static void uart_cr_init(void);
static void uart_cr_start(void);
static void uart_cr_stop(void);

static void uart_dma_init(void);
static void uart_dma_tx_init(void);
static void uart_dma_tx_start(uint16_t len);
static void uart_dma_rx_init(void);
static void uart_dma_rx_start(void);
