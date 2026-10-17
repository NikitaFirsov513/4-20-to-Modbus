#include "FreeRTOS.h"
#include "task.h"
#include "Health.h"
void vApplicationIdleHook(void) { Health_Idle(); }
void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task; (void)name; Health_Panic(FAULT_STACK);
}
void vApplicationMallocFailedHook(void) { Health_Panic(FAULT_HEAP); }
