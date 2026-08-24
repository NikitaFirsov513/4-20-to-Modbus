#ifndef MODBUS_REGISTERS_H
#define MODBUS_REGISTERS_H

#include "App.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f103xb.h"

#define MB_READ_HOLDING_REGISTERS   0x03U
#define MB_WRITE_SINGLE_REGISTER    0x06U
#define MB_WRITE_MULTIPLE_REGISTERS 0x10U

#define MB_REG_SAVE_CONFIG          0x0000U // RW
#define MB_REG_SLAVE_ADDRESS        0x0001U // RW
#define MB_REG_BAUDRATE_CODE        0x0002U // RW
#define MB_REG_PARITY_CODE          0x0003U // RW
#define MB_REG_STOP_BITS            0x0004U // RW

#define MB_REG_CURRENT_UA           0x0100U // RO
#define MB_REG_ADC_MV               0x0101U // RO
#define MB_REG_ADC_RAW              0x0102U // RO
#define MB_REG_STATUS               0x0103U // RO

#define MB_EX_NONE                  0U
#define MB_EX_ILLEGAL_FUNCTION      1U
#define MB_EX_ILLEGAL_DATA_ADDRESS  2U
#define MB_EX_ILLEGAL_DATA_VALUE    3U
#define MB_EX_SLAVE_DEVICE_FAILURE  4U

#define MB_BAUD_1200                0U
#define MB_BAUD_2400                1U
#define MB_BAUD_4800                2U
#define MB_BAUD_9600                3U
#define MB_BAUD_19200               4U
#define MB_BAUD_38400               5U
#define MB_BAUD_57600               6U
#define MB_BAUD_115200              7U

#define MB_PARITY_NONE              0U
#define MB_PARITY_EVEN              1U
#define MB_PARITY_ODD               2U

typedef struct
{   
    uint16_t slave_address;
    uint16_t baudrate_code;
    uint16_t parity_code;
    uint16_t stop_bits;
} connection_data_t;

typedef struct
{
    uint16_t current_uA;
    uint16_t adc_mv;
    uint16_t adc_raw;
    uint16_t status;
} ADC_data_t;

extern connection_data_t connection_data_active;
extern connection_data_t connection_data;
extern ADC_data_t ADC_data;

void modbus_registers_init(void);

uint8_t modbus_registers_read_one(uint16_t reg_address, uint16_t *value);
bool modbus_registers_can_write(uint16_t reg_address);
uint8_t modbus_registers_write_one(uint16_t reg_address, uint16_t value);
uint32_t get_baud_from_code(uint16_t baud_code);

#endif