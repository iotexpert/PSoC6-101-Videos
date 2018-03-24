#include "project.h"
#include "pwmTask.h"
#include "uartTask.h"
#include "ezi2cTask.h"
#include "capsenseTask.h"
#include "bleTask.h"
#include "ezi2cTask.h"
#include <stdio.h>

TaskHandle_t bleTaskHandle;

QueueHandle_t pwmQueue;
EventGroupHandle_t pwmEventGroup;

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    Cy_GPIO_Write(BLUE_PORT,BLUE_NUM,1);
    Cy_GPIO_Write(RED_PORT,RED_NUM,1);
    Cy_GPIO_Write(GREEN_PORT,GREEN_NUM,1);
    
    RED_PWM_Start();
    
    pwmQueue = xQueueCreate(4, sizeof(PWM_Message_t));  
    pwmEventGroup = xEventGroupCreate();
     
    xTaskCreate( uartTask, "UART Task",400,0,2,0);
    xTaskCreate( pwmTask, "PWM Task",400,0,2,0);
    xTaskCreate( ezi2cTask, "EZI2C Task",400,0,2,0);
    xTaskCreate( capsenseTask, "Capsense Task",(2*1024),0,2,0);
    xTaskCreate( bleTask, "BLE Task",(4*1024),0,3,&bleTaskHandle);
    
    vTaskStartScheduler(); // this function never returns

    while(1); // get rid of the compiler warning
}