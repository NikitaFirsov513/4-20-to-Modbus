#pragma once
#include <stdint.h>
#include <stdbool.h>
#define UART_TX_BUFFER_SIZE 256U
#define UART_RX_BUFFER_SIZE 256U
#define UART_PCLK_HZ 64000000UL
bool uart_task_start(void);
void uart_init(uint32_t baudrate, uint16_t parity, uint16_t stop_bits);
bool uart_change_setings(void);
uint8_t uart_send(const uint8_t *data, uint16_t len);
bool uart_is_busy(void);
void uart_irq_handler(void);
void uart_tim2_irq_handler(void);
void uart_dma_tx_irq_handler(void);
uint16_t uart_calculate_us(uint32_t baudrate, uint16_t parity, uint16_t stop_bits);
