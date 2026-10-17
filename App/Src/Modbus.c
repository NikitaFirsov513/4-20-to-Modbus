#include "Modbus.h"
#include "ModbusRegisters.h"
#include "Health.h"

void modbus_init(void) { modbus_registers_init(); }

uint16_t modbus_crc16(const uint8_t *frame, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; ++i) {
        crc ^= frame[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) ? (crc >> 1) ^ 0xA001U : crc >> 1;
    }
    return crc;
}
bool modbus_verify_crc(const uint8_t *frame, uint16_t len)
{
    return frame && len >= 4U &&
        modbus_crc16(frame, len - 2U) ==
        ((uint16_t)frame[len - 2U] | ((uint16_t)frame[len - 1U] << 8));
}
static void append_crc(uint8_t *tx, uint16_t *len)
{
    uint16_t crc = modbus_crc16(tx, *len);
    tx[(*len)++] = (uint8_t)crc;
    tx[(*len)++] = (uint8_t)(crc >> 8);
}
static uint16_t read_be(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }

/* Caller supplies MODBUS_MAX_ADU bytes of response storage. Errors belonging
 * to this slave produce exception frames. Foreign addresses and bad CRC are silent. */
uint8_t modbus_read_frame(uint8_t *frame, uint16_t len, uint8_t *tx, uint16_t *tx_len)
{
    if (!tx_len) return MB_EX_SLAVE_DEVICE_FAILURE;
    *tx_len = 0U;
    if (!frame || !tx || len < 4U || len > MODBUS_MAX_ADU)
        return MB_EX_ILLEGAL_DATA_VALUE;
    if (!modbus_verify_crc(frame,len)) { Health_Increment(COUNT_CRC_ERRORS); return MB_EX_ILLEGAL_DATA_VALUE; }
    bool broadcast = frame[0] == 0U;
    if (!broadcast && frame[0] != connection_data_active.slave_address)
        return MB_EX_ILLEGAL_DATA_ADDRESS;
    uint8_t function = frame[1];
    uint8_t result = MB_EX_ILLEGAL_FUNCTION;
    if (function == MB_READ_HOLDING_REGISTERS || function == MB_WRITE_SINGLE_REGISTER) {
        if (len != 8U) result = MB_EX_ILLEGAL_DATA_VALUE;
        else if (function == MB_READ_HOLDING_REGISTERS) {
            if (broadcast) return MB_EX_NONE;
            result = modbus_read_holding_registers(frame[0], read_be(frame + 2), read_be(frame + 4), tx, tx_len);
        } else result = modbus_write_single_register(frame[0], read_be(frame + 2), read_be(frame + 4), tx, tx_len);
    } else if (function == MB_WRITE_MULTIPLE_REGISTERS) {
        if (len < 9U || len != 9U + frame[6]) result = MB_EX_ILLEGAL_DATA_VALUE;
        else result = modbus_write_multiple_registers(frame[0], read_be(frame + 2), read_be(frame + 4), frame[6], frame + 7, tx, tx_len);
    }
    if (broadcast) *tx_len = 0U;
    else if (result != MB_EX_NONE) {
        Health_Increment(COUNT_MODBUS_EXCEPTIONS);
        tx[0] = frame[0]; tx[1] = function | 0x80U; tx[2] = result;
        *tx_len = 3U;
        append_crc(tx, tx_len);
    }
    return result;
}
uint8_t modbus_read_holding_registers(uint8_t slave, uint16_t address, uint16_t quantity, uint8_t *tx, uint16_t *len)
{
    if (!tx || !len) return MB_EX_SLAVE_DEVICE_FAILURE;
    *len = 0U;
    if (slave == 0U || slave > 247U || quantity == 0U || quantity > 125U)
        return MB_EX_ILLEGAL_DATA_VALUE;
    if ((uint32_t)address + quantity > 65536UL) return MB_EX_ILLEGAL_DATA_ADDRESS;
    uint16_t values[125];
    uint8_t result = modbus_registers_read_block(address,quantity,values);
    if (result != MB_EX_NONE) return result;
    tx[0] = slave; tx[1] = 3U; tx[2] = (uint8_t)(quantity * 2U);
    *len = 3U;
    for (uint16_t i = 0; i < quantity; ++i) {
        tx[(*len)++] = (uint8_t)(values[i] >> 8);
        tx[(*len)++] = (uint8_t)values[i];
    }
    append_crc(tx, len);
    return MB_EX_NONE;
}
uint8_t modbus_write_single_register(uint8_t slave, uint16_t address, uint16_t value, uint8_t *tx, uint16_t *len)
{
    if (!tx || !len) return MB_EX_SLAVE_DEVICE_FAILURE;
    *len = 0U;
    uint8_t result = modbus_registers_write_one(address, value);
    if (result != MB_EX_NONE) return result;
    tx[0] = slave; tx[1] = 6U;
    tx[2] = (uint8_t)(address >> 8); tx[3] = (uint8_t)address;
    tx[4] = (uint8_t)(value >> 8); tx[5] = (uint8_t)value;
    *len = 6U; append_crc(tx, len);
    return MB_EX_NONE;
}
uint8_t modbus_write_multiple_registers(uint8_t slave, uint16_t address, uint16_t quantity, uint8_t bytes, uint8_t *data, uint8_t *tx, uint16_t *len)
{
    if (!tx || !len) return MB_EX_SLAVE_DEVICE_FAILURE;
    *len = 0U;
    if (!data || quantity == 0U || quantity > 123U || bytes != quantity * 2U)
        return MB_EX_ILLEGAL_DATA_VALUE;
    if ((uint32_t)address + quantity > 65536UL) return MB_EX_ILLEGAL_DATA_ADDRESS;
    uint8_t result = modbus_registers_write_many(address, quantity, data);
    if (result != MB_EX_NONE) return result;
    tx[0] = slave; tx[1] = 16U;
    tx[2] = (uint8_t)(address >> 8); tx[3] = (uint8_t)address;
    tx[4] = (uint8_t)(quantity >> 8); tx[5] = (uint8_t)quantity;
    *len = 6U; append_crc(tx, len);
    return MB_EX_NONE;
}
