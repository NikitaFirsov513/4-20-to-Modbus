#include "App.h"
#include "Modbus.h"
#include "SensorMath.h"
#include "stm32f1xx.h"
#include <stddef.h>
#include <string.h>
unsigned checks;
volatile unsigned test_flash_fault, test_fail_after, test_program_index;
volatile uint32_t test_corrupt_address;
#define CHECK(x) do { ++checks; if (!(x)) return __LINE__; } while (0)
static void crc(FlashConfigT *c) { c->crc = modbus_crc16((uint8_t *)c,offsetof(FlashConfigT,crc)); }
int test_main(void)
{
    FlashConfigT a = {1,3,0,1,0}, loaded; crc(&a);
    CHECK(!App_LoadConfig(NULL) && !App_SaveConfig(NULL));
    /* Legacy 10-byte settings at page B are seeded by the test harness. */
    CHECK(App_LoadConfig(&loaded) && loaded.slave_address == 1);
    CHECK(App_StorageFlags() == 5U);
    CHECK(App_SaveConfig(&a));
    CHECK(App_StorageFlags() == 11U);
    CHECK(modbus_verify_crc((const uint8_t *)CONFIG_FLASH_ADDR,10U));
    int32_t low,high; App_LoadCalibration(&low,&high);
    CHECK(low == 2331 && high == 39225);
    CHECK(App_SaveConfig(&a)); /* Same settings must not erase another page. */
    a.slave_address = 2; CHECK(!App_SaveConfig(&a)); crc(&a);
    test_flash_fault=1;
    CHECK(!App_SaveConfig(&a));
    CHECK(App_LoadConfig(&loaded) && loaded.slave_address == 1);
    test_flash_fault=2;
    CHECK(!App_SaveConfig(&a));
    CHECK(App_LoadConfig(&loaded) && loaded.slave_address == 1);
    /* Inject program failure at every halfword, including the final commit.
     * No failed/incomplete record may replace the previously valid page. */
    for (unsigned position=0;position<16U;++position) {
        test_flash_fault=3; test_fail_after=position; test_program_index=0;
        CHECK(!App_SaveConfig(&a));
        CHECK(App_LoadConfig(&loaded) && loaded.slave_address == 1);
        App_LoadCalibration(&low,&high); CHECK(low == 2331 && high == 39225);
    }
    test_flash_fault=0;
    CHECK(App_SaveConfig(&a));
    CHECK(App_LoadConfig(&loaded) && loaded.slave_address == 2);
    a.slave_address=3; crc(&a); CHECK(App_SaveConfig(&a));
    CHECK(App_LoadConfig(&loaded) && loaded.slave_address == 3);
    /* Harness flips a CRC byte in the latest page, as an independent fault. */
    test_corrupt_address=CONFIG_PAGE_A_ADDR+28U;
    CHECK(App_LoadConfig(&loaded) && loaded.slave_address == 2);
    App_LoadCalibration(&low,&high); CHECK(low == 2331 && high == 39225);
    CHECK(FLASH->CR & FLASH_CR_LOCK);
    return 0;
}

/* Entry points for reset-after-write campaigns driven by the ARM harness. */
int powercut_seed(void)
{
    FlashConfigT c={1,3,0,1,0}; crc(&c); return App_SaveConfig(&c) ? 0 : 1;
}
int powercut_update(void)
{
    FlashConfigT c={2,7,2,2,0}; crc(&c); return App_SaveConfig(&c) ? 0 : 1;
}
int powercut_check(void)
{
    FlashConfigT c; int32_t low,high;
    if (!App_LoadConfig(&c)) return 9;
    App_LoadCalibration(&low,&high);
    if (low!=2331 || high!=39225) return 10;
    return c.slave_address;
}
