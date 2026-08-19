#ifndef APP_H
#define APP_H

#define FLASH_KEY1_VALUE  0x45670123UL
#define FLASH_KEY2_VALUE  0xCDEF89ABUL

#define CONFIG_FLASH_ADDR 0x0800FC00UL

#include <stdint.h>

typedef struct {
    uint16_t slave_address;
    uint16_t baudrate_code;
    uint16_t parity_code;
    uint16_t stop_bits;
    uint16_t crc;
} FlashConfigT;

void App_Start(void);

#endif