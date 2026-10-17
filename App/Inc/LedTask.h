#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>

#define QUEUE_SIZE        5U

#define LED_MESSAGE_OK      1U
#define LED_ERROR_CRC       10U
#define LED_ERROR_ADDRESS   11U
#define LED_ERROR_FORMAT    12U
#define LED_ERROR_SPEED     13U

void LedTaskStart(void);

void LedTask_SendMessage(uint8_t message);
void LedTask_SendMessageFromISR(uint8_t message);

#endif