#include "App.h"
#include "Modbus.h"
#include "SensorMath.h"
#include "Health.h"
#include "stm32f1xx.h"
#include <stddef.h>
#include <string.h>

#define STORE_MAGIC 0x4D423432UL
#define STORE_COMMIT 0xA55AU
#define STORE_VERSION 1U
typedef struct {
    uint32_t magic, sequence;
    uint16_t version, length;
    uint16_t address, baud, parity, stop;
    int32_t raw4_x10, raw20_x10;
    uint16_t crc, commit;
} StoreRecord;
_Static_assert(sizeof(StoreRecord) == 32U, "Unexpected persistent record layout");
_Static_assert(offsetof(StoreRecord, crc) == 28U, "Unexpected CRC offset");
static const uint32_t pages[2] = {CONFIG_PAGE_A_ADDR, CONFIG_PAGE_B_ADDR};

static bool settings_valid(const FlashConfigT *c)
{
    return c && c->slave_address >= 1U && c->slave_address <= 247U &&
        c->baudrate_code <= 7U && c->parity_code <= 2U &&
        (c->stop_bits == 1U || c->stop_bits == 2U) &&
        modbus_verify_crc((const uint8_t *)c, sizeof(*c));
}
static void to_config(const StoreRecord *r, FlashConfigT *c)
{
    *c = (FlashConfigT){r->address,r->baud,r->parity,r->stop,0U};
    c->crc = modbus_crc16((const uint8_t *)c, offsetof(FlashConfigT,crc));
}
static bool read_record(unsigned page, StoreRecord *r)
{
    memcpy(r, (const void *)pages[page], sizeof(*r));
    if (r->magic != STORE_MAGIC || r->version != STORE_VERSION ||
        r->length != sizeof(*r) || r->commit != STORE_COMMIT ||
        r->crc != modbus_crc16((const uint8_t *)r, offsetof(StoreRecord,crc)) ||
        r->raw4_x10 < 0 || r->raw20_x10 > 40950 || r->raw20_x10 <= r->raw4_x10) return false;
    FlashConfigT c; to_config(r, &c);
    return settings_valid(&c);
}
static int newest(StoreRecord *record)
{
    StoreRecord a,b;
    bool va = read_record(0U,&a), vb = read_record(1U,&b);
    if (!va && !vb) return -1;
    if (vb && (!va || (int32_t)(b.sequence-a.sequence) > 0)) { *record = b; return 1; }
    *record = a; return 0;
}
static bool legacy(FlashConfigT *config)
{
    memcpy(config,(const void *)CONFIG_FLASH_ADDR,sizeof(*config));
    return settings_valid(config);
}
bool App_LoadConfig(FlashConfigT *config)
{
    if (!config) return false;
    StoreRecord r;
    if (newest(&r) >= 0) { to_config(&r,config); return true; }
    return legacy(config);
}
void App_LoadCalibration(int32_t *low, int32_t *high)
{
    StoreRecord r;
    if (newest(&r) >= 0) { *low = r.raw4_x10; *high = r.raw20_x10; }
    else { *low = SENSOR_CAL_RAW_4MA_X10; *high = SENSOR_CAL_RAW_20MA_X10; }
}
uint16_t App_StorageFlags(void)
{
    StoreRecord r;
    if (newest(&r) >= 0) return 1U | 2U | 8U;
    FlashConfigT c;
    return legacy(&c) ? 1U | 4U : 0U;
}
static bool ready(void)
{
    uint32_t limit = 6400000UL;
    while (FLASH->SR & FLASH_SR_BSY) if (--limit == 0U) return false;
    return true;
}
static void clear_flags(void) { FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR; }
static bool ok(void) { return !(FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)); }
static bool program(uint32_t address, uint16_t value)
{
    if (!ready()) return false;
    clear_flags();
    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint16_t *)address = value;
    bool result = ready();
    FLASH->CR &= ~FLASH_CR_PG;
    return result && ok() && *(volatile uint16_t *)address == value;
}
static bool write_record(unsigned page, const StoreRecord *record)
{
    if (!ready()) return false;
    if (FLASH->CR & FLASH_CR_LOCK) {
        FLASH->KEYR = FLASH_KEY1_VALUE; FLASH->KEYR = FLASH_KEY2_VALUE;
    }
    if (FLASH->CR & FLASH_CR_LOCK) return false;
    clear_flags(); FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = pages[page]; FLASH->CR |= FLASH_CR_STRT;
    bool result = ready();
    FLASH->CR &= ~FLASH_CR_PER;
    result = result && ok();
    const uint8_t *bytes = (const uint8_t *)record;
    for (unsigned i = 0U; result && i < offsetof(StoreRecord,commit); i += 2U)
        result = program(pages[page]+i, bytes[i] | ((uint16_t)bytes[i+1] << 8));
    if (result) result = !memcmp((const void *)pages[page],record,offsetof(StoreRecord,commit));
    if (result) result = program(pages[page]+offsetof(StoreRecord,commit), STORE_COMMIT);
    FLASH->CR |= FLASH_CR_LOCK;
    clear_flags();
    StoreRecord verify;
    return result && read_record(page,&verify) && !memcmp(&verify,record,sizeof(verify));
}
bool App_SaveConfig(const FlashConfigT *config)
{
    if (!settings_valid(config)) return false;
    StoreRecord previous;
    int active = newest(&previous);
    int32_t low,high; Sensor_GetCalibration(&low,&high);
    if (active >= 0 && previous.address == config->slave_address &&
        previous.baud == config->baudrate_code && previous.parity == config->parity_code &&
        previous.stop == config->stop_bits && previous.raw4_x10 == low && previous.raw20_x10 == high) return true;
    StoreRecord record = {
        STORE_MAGIC, active < 0 ? 1U : previous.sequence+1U, STORE_VERSION, sizeof(StoreRecord),
        config->slave_address,config->baudrate_code,config->parity_code,config->stop_bits,
        low,high,0U,STORE_COMMIT
    };
    record.crc = modbus_crc16((const uint8_t *)&record, offsetof(StoreRecord,crc));
    /* First migration writes page A, leaving the legacy record at page B intact. */
    if (!write_record(active < 0 ? 0U : (unsigned)(1-active),&record)) {
        Health_Increment(COUNT_FLASH_FAILURES); return false;
    }
    return true;
}
