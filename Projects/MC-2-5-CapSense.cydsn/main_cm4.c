#include "project.h"
#include "uartTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pwmTask.h"
#include "ezi2cTask.h"
#include "event_groups.h"
#include "capsenseTask.h"

QueueHandle_t pwmQueue;
SemaphoreHandle_t updateEZI2CSemaphore;
EventGroupHandle_t pwmEventGroup;

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    ledblink_Start();

    pwmQueue = xQueueCreate(4, sizeof(PWM_Message_t));  
    updateEZI2CSemaphore = xSemaphoreCreateBinary();
    pwmEventGroup = xEventGroupCreate();
    
    xTaskCreate( uartTask, "UART Task",400,0,2,0);
    xTaskCreate( pwmTask, "PWM Task",400,0,2,0);
    xTaskCreate( ezi2cTask, "EZI2C Task",400,0,2,0);
    xTaskCreate( capsenseTask, "Capsense Task",(2*1024),0,2,0);
    
    vTaskStartScheduler(); // this function never returns
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
