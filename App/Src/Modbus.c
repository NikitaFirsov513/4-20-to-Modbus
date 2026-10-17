#include "Modbus.h"
#include "FreeRTOS.h"
#include "LedTask.h"
#include "ModbusRegisters.h"
#include "UART.h"
#include "queue.h"
void modbus_init(void)
{
    modbus_registers_init();
}

uint16_t modbus_crc16(const uint8_t *frame, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= frame[i];

        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if (crc & 0x0001U)
            {
                crc >>= 1;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

bool modbus_verify_crc(const uint8_t *frame, uint16_t len)
{
    if (len < 4U)
    {
        return false;
    }

    uint16_t received_crc =
        (uint16_t)frame[len - 2U] |
        ((uint16_t)frame[len - 1U] << 8);

    uint16_t calculated_crc = modbus_crc16(frame, len - 2U);

    return received_crc == calculated_crc;
}

uint8_t modbus_read_frame(uint8_t *frame, uint16_t len, uint8_t *tx, uint16_t *tx_len)
{

    uint8_t slave_address = frame[0];

    if (slave_address != connection_data_active.slave_address)
    {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }

    bool crc_valid = modbus_verify_crc(frame, len);

    if (!crc_valid)
    {
        // нужно помегать ошибку CRC
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    uint8_t function_code = frame[1];

    if (function_code == MB_READ_HOLDING_REGISTERS)
    {
        uint16_t start_address = (uint16_t)frame[2] << 8 | frame[3];
        uint16_t quantity = (uint16_t)frame[4] << 8 | frame[5];

        uint8_t result = modbus_read_holding_registers(slave_address, start_address, quantity, tx, tx_len);

        if (result != MB_EX_NONE)
        {

            return result;
        }
        else
        {
            LedTask_SendMessageFromISR(LED_MESSAGE_OK);
        }
    }

    if (function_code == MB_WRITE_SINGLE_REGISTER)
    {
        uint16_t address = (uint16_t)frame[2] << 8 | frame[3];
        uint16_t value = (uint16_t)frame[4] << 8 | frame[5];

        modbus_write_single_register(slave_address, address, value, tx, tx_len);
    }

    if (function_code == MB_WRITE_MULTIPLE_REGISTERS)
    {
        uint16_t start_address = (uint16_t)frame[2] << 8 | frame[3];
        uint16_t quantity = (uint16_t)frame[4] << 8 | frame[5];
        uint8_t data_len = frame[6];
        uint8_t *data = &frame[7];

        modbus_write_multiple_registers(slave_address, start_address, quantity, data_len, data, tx, tx_len);
    }
    return MB_EX_NONE;
}

uint8_t modbus_read_holding_registers(uint8_t slave_addr,
                                      uint16_t start_address,
                                      uint16_t quantity,
                                      uint8_t *tx,
                                      uint16_t *tx_len)
{
    if ((tx == 0) || (tx_len == 0))
    {
        return MB_EX_SLAVE_DEVICE_FAILURE;
    }

    *tx_len = 0U;

    if ((slave_addr == 0U) || (slave_addr > 125U))
    {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    tx[(*tx_len)++] = slave_addr;
    tx[(*tx_len)++] = 0x03U;
    tx[(*tx_len)++] = (uint8_t)(quantity * 2U);

    for (uint16_t i = 0; i < quantity; i++)
    {
        uint16_t value;
        uint16_t reg_address = start_address + i;

        uint8_t result = modbus_registers_read_one(reg_address, &value);

        if (result != MB_EX_NONE)
        {
            return result;
        }

        tx[(*tx_len)++] = (uint8_t)((value >> 8) & 0xFFU);
        tx[(*tx_len)++] = (uint8_t)(value & 0xFFU);
    }

    uint16_t crc = modbus_crc16(tx, *tx_len);
    tx[(*tx_len)++] = (uint8_t)(crc & 0xFFU);        // CRC low byte
    tx[(*tx_len)++] = (uint8_t)((crc >> 8) & 0xFFU); // CRC high byte

    return MB_EX_NONE;
}

uint8_t modbus_write_single_register(uint8_t slave_addr,
                                     uint16_t reg_addr,
                                     uint16_t value,
                                     uint8_t *tx,
                                     uint16_t *tx_len)
{
    if ((tx == 0) || (tx_len == 0))
    {
        return MB_EX_SLAVE_DEVICE_FAILURE;
    }

    *tx_len = 0U;

    bool can_write = modbus_registers_can_write(reg_addr);

    if (!can_write)
    {
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    }

    uint8_t res = modbus_registers_write_one(reg_addr, value);
    if (res != MB_EX_NONE)
    {
        return res;
    }

    tx[(*tx_len)++] = slave_addr;
    tx[(*tx_len)++] = 0x06U;
    tx[(*tx_len)++] = (uint8_t)((reg_addr >> 8) & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)(reg_addr & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)((value >> 8) & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)(value & 0xFFU);
    uint16_t crc = modbus_crc16(tx, *tx_len);
    tx[(*tx_len)++] = (uint8_t)(crc & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)((crc >> 8) & 0xFFU);

    return MB_EX_NONE;
}

uint8_t modbus_write_multiple_registers(uint8_t slave_addr,
                                        uint16_t start_address,
                                        uint16_t quantity,
                                        uint8_t data_len,
                                        uint8_t *data,
                                        uint8_t *tx,
                                        uint16_t *tx_len)
{
    if ((tx == 0) || (tx_len == 0))
    {
        return MB_EX_SLAVE_DEVICE_FAILURE;
    }

    if (data_len != quantity * 2)
    {
        return MB_EX_ILLEGAL_DATA_VALUE;
    }

    for (uint16_t i = 0; i < quantity; i++)
    {
        uint16_t reg_address =
            start_address + i;

        uint16_t value =
            ((uint16_t)data[i * 2] << 8) |
            data[i * 2 + 1];

        if (!modbus_registers_can_write(reg_address))
        {
            return MB_EX_ILLEGAL_DATA_ADDRESS;
        }
    }

    for (uint16_t i = 0; i < quantity; i++)
    {
        uint16_t reg_address = start_address + i;
        uint16_t value = (uint16_t)data[i * 2] << 8 | data[i * 2 + 1];
        uint8_t res = modbus_registers_write_one(reg_address, value);
        if (res != MB_EX_NONE)
        {
            return res;
        }
    }

    *tx_len = 0U;

    tx[(*tx_len)++] = slave_addr;
    tx[(*tx_len)++] = 0x10U;
    tx[(*tx_len)++] = (uint8_t)((start_address >> 8) & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)(start_address & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)((quantity >> 8) & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)(quantity & 0xFFU);
    uint16_t crc = modbus_crc16(tx, *tx_len);
    tx[(*tx_len)++] = (uint8_t)(crc & 0xFFU);
    tx[(*tx_len)++] = (uint8_t)((crc >> 8) & 0xFFU);

    return MB_EX_NONE;
}