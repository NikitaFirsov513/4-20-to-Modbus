#include "AdcTask.h"
#include "FreeRTOS.h"
#include "ModbusRegisters.h"
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "stdint.h"
#include "task.h"
static uint16_t adc;

uint16_t ADC1_Read(void)
{

    LL_ADC_REG_StartConversionSWStart(ADC1);

    while (!LL_ADC_IsActiveFlag_EOS(ADC1))
    {
    }

    return LL_ADC_REG_ReadConversionData12(ADC1);
}

void AdcTask(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    for (;;)
    {

        adc = ADC1_Read();

        float voltage = (float)adc * 3.3f / 4095.0f;

        float current_mA = 4.0f + (voltage - 0.17f) * 16.0f / (3.17f - 0.17f);

        ADC_data.current_uA = current_mA * 1000;
        ADC_data.adc_mv = voltage * 1000;
        ADC_data.adc_raw = adc;
        ADC_data.status = 0U;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
    }
}