/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "project.h"
#include "uartTask.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pwmTask.h"

QueueHandle_t pwmQueue;

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    pwmQueue = xQueueCreate(4, sizeof(PWM_Message_t));  
    
    xTaskCreate( uartTask, "UART Task",400,0,2,0);
    xTaskCreate( pwmTask, "PWM Task",400,0,2,0);
    
    vTaskStartScheduler(); // this function never returns
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
