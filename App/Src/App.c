#include "App.h"
#include "AdcTask.h"
#include "FreeRTOS.h"
#include "LedTask.h"
#include "Modbus.h"
#include "ModbusRegisters.h"
#include "Sensor.h"
#include "UART.h"
#include "main.h"
#include "task.h"
FlashConfigT *config_flash_ptr = (FlashConfigT *)CONFIG_FLASH_ADDR;

static void flash_wait_ready(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0U)
    {
    }
}

// ============================================================================
// Очистить флаги
//
// В STM32F1 эти флаги сбрасываются записью 1.
// ============================================================================

static void flash_clear_flags(void)
{
    FLASH->SR =
        FLASH_SR_EOP |
        FLASH_SR_PGERR |
        FLASH_SR_WRPRTERR;
}

// ============================================================================
// Проверка ошибок Flash
// ============================================================================

static bool flash_has_error(void)
{
    if ((FLASH->SR & FLASH_SR_PGERR) != 0U)
    {
        return true;
    }

    if ((FLASH->SR & FLASH_SR_WRPRTERR) != 0U)
    {
        return true;
    }

    return false;
}

// ============================================================================
// Unlock
// ============================================================================

static bool flash_unlock(void)
{
    flash_wait_ready();

    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        FLASH->KEYR = FLASH_KEY1_VALUE;
        FLASH->KEYR = FLASH_KEY2_VALUE;
    }

    // Если LOCK остался — unlock не удался
    if ((FLASH->CR & FLASH_CR_LOCK) != 0U)
    {
        return false;
    }

    return true;
}

// ============================================================================
// Lock
// ============================================================================

static void flash_lock(void)
{
    flash_wait_ready();

    FLASH->CR |= FLASH_CR_LOCK;
}

// ============================================================================
// Стирание одной страницы
// ============================================================================

static bool flash_erase_page(uint32_t page_address)
{
    flash_wait_ready();

    flash_clear_flags();

    // Выбираем Page Erase
    FLASH->CR |= FLASH_CR_PER;

    // Адрес страницы
    FLASH->AR = page_address;

    // Запускаем стирание
    FLASH->CR |= FLASH_CR_STRT;

    // Ждём
    flash_wait_ready();

    // Отключаем режим Page Erase
    FLASH->CR &= ~FLASH_CR_PER;

    // Проверяем ошибки
    if (flash_has_error())
    {
        flash_clear_flags();

        return false;
    }

    // Сбрасываем EOP
    if ((FLASH->SR & FLASH_SR_EOP) != 0U)
    {
        FLASH->SR = FLASH_SR_EOP;
    }

    return true;
}

// ============================================================================
// Записать один HALF-WORD = 16 бит
// ============================================================================

static bool flash_program_halfword(
    uint32_t address,
    uint16_t value)
{
    flash_wait_ready();

    flash_clear_flags();

    // Включаем режим программирования
    FLASH->CR |= FLASH_CR_PG;

    // ========================================================
    // ВАЖНО:
    //
    // FLASH->AR здесь НЕ используется.
    //
    // При программировании STM32F1 нужно просто выполнить
    // запись uint16_t прямо по адресу Flash.
    // ========================================================

    *(volatile uint16_t *)address = value;

    // Ждём завершения
    flash_wait_ready();

    // Выключаем programming mode
    FLASH->CR &= ~FLASH_CR_PG;

    // Ошибки?
    if (flash_has_error())
    {
        flash_clear_flags();

        return false;
    }

    // Проверяем записанное значение
    if (*(volatile uint16_t *)address != value)
    {
        return false;
    }

    // EOP
    if ((FLASH->SR & FLASH_SR_EOP) != 0U)
    {
        FLASH->SR = FLASH_SR_EOP;
    }

    return true;
}

bool save_default_connection_data(
    FlashConfigT *config_flash_ptr)
{
    FlashConfigT default_settings = {};

    default_settings.slave_address = 1U;

    default_settings.baudrate_code =
        MB_BAUD_9600;

    default_settings.parity_code =
        MB_PARITY_NONE;

    default_settings.stop_bits =
        1U;

    default_settings.crc =
        modbus_crc16(
            (uint8_t *)&default_settings,
            offsetof(FlashConfigT, crc));

    if (!flash_unlock())
    {
        return false;
    }

    if (!flash_erase_page(
            (uint32_t)config_flash_ptr))
    {
        flash_lock();

        return false;
    }

    const uint16_t *data =
        (const uint16_t *)&default_settings;

    uint32_t address =
        (uint32_t)config_flash_ptr;

    uint32_t halfword_count =
        sizeof(FlashConfigT) /
        sizeof(uint16_t);

    for (uint32_t i = 0U;
         i < halfword_count;
         i++)
    {
        if (!flash_program_halfword(
                address,
                data[i]))
        {
            flash_lock();

            return false;
        }

        address += 2U;
    }

    flash_lock();

    if (!modbus_verify_crc(
            (uint8_t *)config_flash_ptr,
            sizeof(FlashConfigT)))
    {
        return false;
    }

    return true;
}

void App_Start(void)
{

    Sensor_TIM3_Init();
    Sensor_ADC_Init();
    Sensor_DMA_Init();

    Sensor_ADC_Start();
    Sensor_DMA_Start();
    Sensor_TIM3_Start();

    bool is_settings_mem = modbus_verify_crc((uint8_t *)config_flash_ptr, 10U);
    if (is_settings_mem)
    {
        uart_init(get_baud_from_code(config_flash_ptr->baudrate_code), config_flash_ptr->parity_code, config_flash_ptr->stop_bits);
    }
    else
    {
        save_default_connection_data(config_flash_ptr);
        uart_init(get_baud_from_code(config_flash_ptr->baudrate_code), config_flash_ptr->parity_code, config_flash_ptr->stop_bits);
    }

    modbus_init();

    xTaskCreate(AdcTask, "adc", 256, NULL, 2, NULL);
    LedTaskStart();
    vTaskStartScheduler();

    for (;;)
    {
    }
    // Error_Handler();
}