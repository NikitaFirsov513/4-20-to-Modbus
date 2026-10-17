#include "LedTask.h"
#include "Health.h"
#include "ModbusRegisters.h"

#include "stm32f103xb.h"
#include "main.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

static QueueHandle_t messageQueue = NULL;
static uint16_t call_col = 0U;

static void LedTask(void *pvParameters);

static void blinkMessage(uint8_t message);

static void resetLight(void);
static void setRedLight(void);
static void setBlueLight(void);
static void setGreenLight(void);

void LedTaskStart(void)
{
    call_col = 0U;

    messageQueue = xQueueCreate(QUEUE_SIZE, sizeof(uint8_t));

    TaskHandle_t handle;
    if (!messageQueue || xTaskCreate(LedTask, "led", 256, NULL, 2, &handle) != pdPASS)
        Error_Handler();
    Health_RegisterTask(HEALTH_LED,handle);
}

void LedTask_SendMessage(uint8_t message)
{
    if (messageQueue != NULL)
    {
        (void)xQueueSend(messageQueue, &message, pdMS_TO_TICKS(10));
    }
}

void LedTask_SendMessageFromISR(uint8_t message)
{
    if (messageQueue != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        (void)xQueueSendFromISR(messageQueue,
                                &message,
                                &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

static void LedTask(void *pvParameters)
{
    (void)pvParameters;

    uint8_t message = 0U;

    for (;;)
    {
        Health_Heartbeat(HEALTH_LED);
        if (xQueueReceive(messageQueue, &message, pdMS_TO_TICKS(100)) == pdPASS)
        {
            blinkMessage(message);
            call_col = 0U;
        }
        else
         {
             if (call_col > 10U)
             {
                 resetLight();

                 ADC_data_t adc;
                 modbus_snapshot_adc(&adc);
                 if (adc.status & (ADC_STATUS_STALE | ADC_STATUS_DMA_ERROR | ADC_STATUS_LOOP_OPEN | ADC_STATUS_SATURATED))
                     setRedLight();
                 else if (App_DefaultsForced()) setBlueLight();
                 else setGreenLight();
                 vTaskDelay(pdMS_TO_TICKS(600));

                 resetLight();

                 call_col = 0U;
             }

             call_col++;
         }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

static void blinkMessage(uint8_t message)
{
    resetLight();

    switch (message)
    {
    case LED_MESSAGE_OK: {
        setGreenLight();
        vTaskDelay(pdMS_TO_TICKS(100));
        resetLight();
        break;
    }

    case LED_ERROR_CRC:
    case LED_ERROR_ADDRESS:
    case LED_ERROR_FORMAT:
        setRedLight();
        vTaskDelay(pdMS_TO_TICKS(150));
        resetLight();
        break;
    case LED_ERROR_SPEED:
        setBlueLight();
        vTaskDelay(pdMS_TO_TICKS(150));
        resetLight();
        break;

    default: {
        break;
    }
    }
}

static void resetLight(void)
{
    GPIOB->BRR = GPIO_BRR_BR3;
    GPIOB->BRR = GPIO_BRR_BR4;
    GPIOB->BRR = GPIO_BRR_BR5;
}

static void setRedLight(void)
{
    GPIOB->BSRR = GPIO_BSRR_BS5;
}

static void setGreenLight(void)
{
    GPIOB->BSRR = GPIO_BSRR_BS4;
}

static void setBlueLight(void)
{
    GPIOB->BSRR = GPIO_BSRR_BS3;
}