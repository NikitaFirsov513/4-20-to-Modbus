#include "AdcTask.h"
#include "Sensor.h"
#include "SensorMath.h"
#include "ModbusRegisters.h"
#include "Health.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx.h"
void AdcTask(void *argument)
{
    (void)argument;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t previous_sequence = 0U;
    unsigned missing = 0U;
    SensorStatusFilter filter = {0};
    for (;;) {
        uint16_t raw; uint32_t sequence; bool dma_error;
        Sensor_Read(&raw,&sequence,&dma_error);
        ADC_data_t data;
        Sensor_Convert(raw,(GPIOA->IDR & GPIO_IDR_IDR1) != 0U,&data);
        bool fresh = sequence != previous_sequence && !dma_error;
        if (!fresh) { data.status |= ADC_STATUS_STALE; ++missing; }
        else { missing = 0U; Health_Heartbeat(HEALTH_ADC); }
        if (dma_error) data.status |= ADC_STATUS_DMA_ERROR;
        if (fresh) Sensor_FilterStatus(&filter,&data);
        previous_sequence = sequence;
        modbus_publish_adc(&data);
        if (dma_error || missing >= 3U) {
            Sensor_Recover();
            missing = 0U;
        }
        vTaskDelayUntil(&last_wake,pdMS_TO_TICKS(100));
    }
}
