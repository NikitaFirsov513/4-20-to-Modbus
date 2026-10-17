#include "App.h"
#include "AdcTask.h"
#include "Health.h"
#include "FreeRTOS.h"
#include "LedTask.h"
#include "Modbus.h"
#include "ModbusRegisters.h"
#include "Sensor.h"
#include "SensorMath.h"
#include "UART.h"
#include "main.h"
#include "task.h"

static bool force_defaults;
bool App_DefaultsForced(void) { return force_defaults; }
void App_Start(void)
{
    Health_Init();
    /* SW1 connects PA15 to GND. Sample only at boot, after settling/debounce. */
    HAL_Delay(20U);
    unsigned low = 0U;
    for (unsigned i = 0; i < 10U; ++i) {
        if (!(GPIOA->IDR & GPIO_IDR_IDR15)) ++low;
        HAL_Delay(2U);
    }
    force_defaults = low >= 8U;
    int32_t raw4,raw20;
    App_LoadCalibration(&raw4,&raw20);
    if (!Sensor_SetCalibration(raw4,raw20)) Error_Handler();
    modbus_init();
    HAL_Delay(50U);
    LedTaskStart();
    TaskHandle_t adc_handle;
    if (!uart_task_start() || xTaskCreate(AdcTask,"adc",256U,NULL,2U,&adc_handle) != pdPASS)
        Error_Handler();
    Health_RegisterTask(HEALTH_ADC,adc_handle);
    Sensor_TIM3_Init(); Sensor_ADC_Init(); Sensor_DMA_Init();
    Sensor_ADC_Start(); Sensor_DMA_Start(); Sensor_TIM3_Start();
    uart_init(get_baud_from_code(connection_data_active.baudrate_code),
              connection_data_active.parity_code,connection_data_active.stop_bits);
    Health_StartWatchdog();
    vTaskStartScheduler();
    Error_Handler();
}
