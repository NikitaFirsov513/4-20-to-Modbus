#include "ModbusRegisters.h"
#include "Modbus.h"
#include "Health.h"
#include "SensorMath.h"
#include "UART.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

connection_data_t connection_data;
connection_data_t connection_data_active;
ADC_data_t ADC_data;
static connection_data_t pending_config;
static bool pending;
static bool have_adc;
static TickType_t adc_updated;

bool modbus_config_valid(const connection_data_t *c)
{
    return c && c->slave_address >= 1U && c->slave_address <= 247U &&
        c->baudrate_code <= MB_BAUD_115200 && c->parity_code <= MB_PARITY_ODD &&
        (c->stop_bits == 1U || c->stop_bits == 2U);
}
void modbus_registers_init(void)
{
    connection_data = (connection_data_t){1U, MB_BAUD_9600, MB_PARITY_NONE, 1U};
    FlashConfigT saved;
    if (!App_DefaultsForced() && App_LoadConfig(&saved)) {
        connection_data_t c = {saved.slave_address, saved.baudrate_code, saved.parity_code, saved.stop_bits};
        if (modbus_config_valid(&c)) connection_data = c;
    }
    connection_data_active = connection_data;
    ADC_data = (ADC_data_t){0U, 0U, 0U, ADC_STATUS_STALE};
    pending = false;
    have_adc = false; adc_updated = 0U;
}
bool modbus_registers_can_write(uint16_t address) { return address <= MB_REG_STOP_BITS; }

static uint8_t stage_value(connection_data_t *c, uint16_t address, uint16_t value)
{
    switch (address) {
    case MB_REG_SAVE_CONFIG: return value == 1U ? MB_EX_NONE : MB_EX_ILLEGAL_DATA_VALUE;
    case MB_REG_SLAVE_ADDRESS: if (value < 1U || value > 247U) return MB_EX_ILLEGAL_DATA_VALUE; c->slave_address = value; break;
    case MB_REG_BAUDRATE_CODE: if (value > MB_BAUD_115200) return MB_EX_ILLEGAL_DATA_VALUE; c->baudrate_code = value; break;
    case MB_REG_PARITY_CODE: if (value > MB_PARITY_ODD) return MB_EX_ILLEGAL_DATA_VALUE; c->parity_code = value; break;
    case MB_REG_STOP_BITS: if (value != 1U && value != 2U) return MB_EX_ILLEGAL_DATA_VALUE; c->stop_bits = value; break;
    default: return MB_EX_ILLEGAL_DATA_ADDRESS;
    }
    return MB_EX_NONE;
}
uint8_t modbus_registers_write_many(uint16_t address, uint16_t quantity, const uint8_t *data)
{
    if (!data || !quantity || quantity > 123U) return MB_EX_ILLEGAL_DATA_VALUE;
    if ((uint32_t)address + quantity > MB_REG_STOP_BITS + 1U) return MB_EX_ILLEGAL_DATA_ADDRESS;
    connection_data_t staged = connection_data;
    bool save = false;
    for (uint16_t i = 0U; i < quantity; ++i) {
        uint16_t value = ((uint16_t)data[2U*i] << 8) | data[2U*i+1U];
        uint8_t result = stage_value(&staged, address + i, value);
        if (result != MB_EX_NONE) return result;
        if (address + i == MB_REG_SAVE_CONFIG) save = true;
    }
    if (!modbus_config_valid(&staged)) return MB_EX_ILLEGAL_DATA_VALUE;
    if (save) {
        FlashConfigT flash = {staged.slave_address, staged.baudrate_code, staged.parity_code, staged.stop_bits, 0U};
        flash.crc = modbus_crc16((const uint8_t *)&flash, offsetof(FlashConfigT, crc));
        if (!App_SaveConfig(&flash)) return MB_EX_SLAVE_DEVICE_FAILURE;
        pending_config = staged;
        pending = true;
    }
    /* Commit only after every value and the optional Flash write succeeded. */
    connection_data = staged;
    return MB_EX_NONE;
}
uint8_t modbus_registers_write_one(uint16_t address, uint16_t value)
{
    uint8_t bytes[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    return modbus_registers_write_many(address, 1U, bytes);
}
void modbus_apply_pending_config(void)
{
    if (pending && App_DefaultsForced()) { pending = false; return; }
    if (pending && !uart_is_busy()) {
        connection_data_active = pending_config;
        (void)uart_change_setings();
        pending = false;
    }
}
void modbus_cancel_pending_config(void) { pending = false; }
void modbus_publish_adc(const ADC_data_t *data)
{
    taskENTER_CRITICAL();
    ADC_data = *data;
    if (!(data->status & (ADC_STATUS_STALE | ADC_STATUS_DMA_ERROR))) {
        adc_updated = xTaskGetTickCount(); have_adc = true;
    }
    taskEXIT_CRITICAL();
}
void modbus_snapshot_adc(ADC_data_t *data)
{
    taskENTER_CRITICAL();
    *data = ADC_data;
    if (!have_adc || (TickType_t)(xTaskGetTickCount()-adc_updated) > pdMS_TO_TICKS(300U))
        data->status |= ADC_STATUS_STALE;
    taskEXIT_CRITICAL();
}
uint32_t modbus_adc_age_ms(void)
{
    taskENTER_CRITICAL();
    uint32_t age = have_adc ? (uint32_t)(xTaskGetTickCount()-adc_updated) : UINT32_MAX;
    taskEXIT_CRITICAL();
    return age;
}
static uint16_t half(uint32_t value, bool low) { return low ? (uint16_t)value : (uint16_t)(value >> 16); }
uint8_t modbus_registers_read_block(uint16_t address, uint16_t quantity, uint16_t *values)
{
    if (!values) return MB_EX_SLAVE_DEVICE_FAILURE;
    if (!quantity || (uint32_t)address+quantity > 65536UL) return MB_EX_ILLEGAL_DATA_ADDRESS;
    ADC_data_t adc; modbus_snapshot_adc(&adc);
    /* Freeze changing diagnostics once for the complete RTU response, including
     * both halves of every 32-bit counter and the measurement age. */
    HealthSnapshot health = {0};
    bool diagnostic = (uint32_t)address+quantity > 0x0200U && address < 0x0240U;
    uint32_t age = 0U; uint16_t flags = 0U;
    if (diagnostic) {
        Health_Snapshot(&health); age = modbus_adc_age_ms();
        flags = (App_DefaultsForced() ? 1U : 0U) | ((App_StorageFlags() & 1U) ? 2U : 0U) | 4U;
        if (!(adc.status & (ADC_STATUS_STALE | ADC_STATUS_DMA_ERROR))) flags |= 8U;
        if (App_StorageFlags() & 2U) flags |= 16U;
        if (App_StorageFlags() & 4U) flags |= 32U;
    }
    for (uint16_t i = 0U; i < quantity; ++i) {
        uint16_t reg = address+i, value;
        switch (reg) {
        case MB_REG_SAVE_CONFIG: value = 0U; break;
        case MB_REG_SLAVE_ADDRESS: value = connection_data.slave_address; break;
        case MB_REG_BAUDRATE_CODE: value = connection_data.baudrate_code; break;
        case MB_REG_PARITY_CODE: value = connection_data.parity_code; break;
        case MB_REG_STOP_BITS: value = connection_data.stop_bits; break;
        case 0x0010: value = connection_data_active.slave_address; break;
        case 0x0011: value = connection_data_active.baudrate_code; break;
        case 0x0012: value = connection_data_active.parity_code; break;
        case 0x0013: value = connection_data_active.stop_bits; break;
        case 0x0030: case 0x0031: {
            int32_t low,high; Sensor_GetCalibration(&low,&high);
            value = (uint16_t)(reg == 0x0030 ? low : high); break;
        }
        case MB_REG_CURRENT_UA: value = adc.current_uA; break;
        case MB_REG_ADC_MV: value = adc.adc_mv; break;
        case MB_REG_ADC_RAW: value = adc.adc_raw; break;
        case MB_REG_STATUS: value = adc.status; break;
        case 0x0200: value = 1U; break; /* Firmware 1.1.1. */
        case 0x0201: value = 1U; break;
        case 0x0202: value = 1U; break;
        case 0x0203: value = 0U; break; /* PCB revision not encoded/verified. */
        case 0x0204: value = flags; break;
        case 0x0205: case 0x0206: value = half(health.reset_flags,reg == 0x0206); break;
        case 0x0207: value = health.last_fault; break;
        case 0x0208: case 0x0209: value = half(health.uptime_s,reg == 0x0209); break;
        case 0x020A: case 0x020B: value = half(age,reg == 0x020B); break;
        case 0x020C: value = health.min_heap; break;
        case 0x020D: value = health.free_heap; break;
        case 0x020E: value = health.stack[HEALTH_ADC]; break;
        case 0x020F: value = health.stack[HEALTH_UART]; break;
        case 0x0210: value = health.stack[HEALTH_LED]; break;
        case 0x0217: value = health.alive; break;
        default:
            if (reg >= 0x0211 && reg <= 0x0216) value = half(health.uid[(reg-0x0211)/2], (reg-0x0211)&1U);
            else if (reg >= 0x0220 && reg < 0x0220+2U*HEALTH_COUNTERS)
                value = half(health.counters[(reg-0x0220)/2], (reg-0x0220)&1U);
            else return MB_EX_ILLEGAL_DATA_ADDRESS;
            break;
        }
        values[i] = value;
    }
    return MB_EX_NONE;
}
uint8_t modbus_registers_read_one(uint16_t address, uint16_t *value)
{
    return modbus_registers_read_block(address,1U,value);
}
uint32_t get_baud_from_code(uint16_t code)
{
    static const uint32_t baud[] = {1200U, 2400U, 4800U, 9600U, 19200U, 38400U, 57600U, 115200U};
    return code < sizeof(baud)/sizeof(baud[0]) ? baud[code] : 0U;
}
