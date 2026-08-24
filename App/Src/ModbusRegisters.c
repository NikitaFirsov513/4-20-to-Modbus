#include "ModbusRegisters.h"
#include "UART.h"
#include <stdbool.h>
#include <stdint.h>
connection_data_t connection_data;
connection_data_t connection_data_active;
ADC_data_t ADC_data;

void modbus_registers_init(void)
{
    connection_data.slave_address = 1U;
    connection_data.baudrate_code = 3U; // 115200
    connection_data.parity_code = 0U;   // None
    connection_data.stop_bits = 1U;     // 1 stop bit

    connection_data_active.slave_address = 1U;
    connection_data_active.baudrate_code = 3U; // 115200
    connection_data_active.parity_code = 0U;   // None
    connection_data_active.stop_bits = 1U;     // 1 stop bit

    ADC_data.current_uA = 0U;
    ADC_data.adc_mv = 0U;
    ADC_data.adc_raw = 0U;
    ADC_data.status = 0U;
}

bool modbus_registers_can_write(uint16_t reg_address)
{
    switch (reg_address)
    {
    case MB_REG_SAVE_CONFIG:
    case MB_REG_SLAVE_ADDRESS:
    case MB_REG_BAUDRATE_CODE:
    case MB_REG_PARITY_CODE:
    case MB_REG_STOP_BITS:
        return true;

    default:
        return false;
    }
}

uint8_t modbus_registers_write_one(uint16_t reg_address, uint16_t value)
{
    switch (reg_address)
    {
    case MB_REG_SAVE_CONFIG: {
        // Save configuration to non-volatile memory
        // нужно сделать чтобы устанавливать 1
        // проверка логичности
        connection_data_active = connection_data;
        uart_change_setings();

        return MB_EX_NONE;
    }

    case MB_REG_SLAVE_ADDRESS: {
        connection_data.slave_address = value;
        return MB_EX_NONE;
    }

    case MB_REG_BAUDRATE_CODE: {
        connection_data.baudrate_code = value;
        return MB_EX_NONE;
    }

    case MB_REG_PARITY_CODE: {
        connection_data.parity_code = value;
        return MB_EX_NONE;
    }

    case MB_REG_STOP_BITS: {
        connection_data.stop_bits = value;
        return MB_EX_NONE;
    }

    default: {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    }
}

uint8_t modbus_registers_read_one(uint16_t reg_address, uint16_t *value)
{
    if (value == 0)
    {
        return MB_EX_SLAVE_DEVICE_FAILURE;
    }

    switch (reg_address)
    {

    case MB_REG_SAVE_CONFIG: {
        *value = 0U;
        return MB_EX_NONE;
    }

    case MB_REG_SLAVE_ADDRESS: {
        *value = connection_data.slave_address;
        return MB_EX_NONE;
    }

    case MB_REG_BAUDRATE_CODE: {
        *value = connection_data.baudrate_code;
        return MB_EX_NONE;
    }

    case MB_REG_PARITY_CODE: {
        *value = connection_data.parity_code;
        return MB_EX_NONE;
    }

    case MB_REG_STOP_BITS: {
        *value = connection_data.stop_bits;
        return MB_EX_NONE;
    }

    /*
     * 100.
     */
    case MB_REG_CURRENT_UA: {
        *value = ADC_data.current_uA;
        return MB_EX_NONE;
    }

    case MB_REG_ADC_MV: {
        *value = ADC_data.adc_mv;
        return MB_EX_NONE;
    }

    case MB_REG_ADC_RAW: {
        *value = ADC_data.adc_raw;
        return MB_EX_NONE;
    }

    case MB_REG_STATUS: {
        *value = ADC_data.status;
        return MB_EX_NONE;
    }

    default: {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    }
}

uint32_t get_baud_from_code(uint16_t baud_code)
{
    uint32_t baud = 0;
    switch (baud_code)
    {
    case MB_BAUD_1200:
        baud = 1200;
        /* code */
        break;
    case MB_BAUD_2400:
        /* code */
        baud = 2400;

        break;
    case MB_BAUD_4800:
        /* code */
        baud = 4800;

        break;
    case MB_BAUD_9600:
        baud = 9600;

        /* code */
        break;
    case MB_BAUD_19200:
        baud = 19200;

        /* code */
        break;
    case MB_BAUD_38400:
        baud = 38400;

        /* code */
        break;
    case MB_BAUD_57600:
        baud = 57600;

        /* code */
        break;
    case MB_BAUD_115200:
        baud = 115200;

        /* code */
        break;

    default:
        break;
    }
    return baud;
}