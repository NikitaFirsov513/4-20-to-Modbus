#ifndef APP_H
#define APP_H

#define FLASH_KEY1_VALUE  0x45670123UL
#define FLASH_KEY2_VALUE  0xCDEF89ABUL

#define CONFIG_FLASH_ADDR 0x0800FC00UL /* Legacy record address. */
#define CONFIG_PAGE_A_ADDR 0x0800F800UL
#define CONFIG_PAGE_B_ADDR 0x0800FC00UL
#define CONFIG_PAGE_SIZE 1024UL

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t slave_address;
    uint16_t baudrate_code;
    uint16_t parity_code;
    uint16_t stop_bits;
    uint16_t crc;
} FlashConfigT;

void App_Start(void);
bool App_LoadConfig(FlashConfigT *config);
bool App_SaveConfig(const FlashConfigT *config);
void App_LoadCalibration(int32_t *raw4_x10, int32_t *raw20_x10);
uint16_t App_StorageFlags(void);
bool App_DefaultsForced(void);

#endif
