#include "project.h"

#include "pwmTask.h"
#include "global.h"

#include <stdio.h>

SemaphoreHandle_t updateEZI2CSemaphore;
SemaphoreHandle_t uartSemaphore;
QueueHandle_t pwmQueue;
EventGroupHandle_t pwmEventGroup;


// These forward declarations are 
extern void uartTask(void *arg);       // uartTask.c
extern void pwmTask(void *arg);        // pwmTask.c
extern void ezi2cTask(void *arg);      // ezi2cTask.c
extern void capsenseTask(void *arg);   // capsenseTask.c
extern void bleTask(void *arg);        // bleTask.c

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    Cy_GPIO_Write(BLUE_PORT,BLUE_NUM,1);
    Cy_GPIO_Write(RED_PORT,RED_NUM,1);
    Cy_GPIO_Write(GREEN_PORT,GREEN_NUM,1);
    
    UART_Start();
    RED_PWM_Start();
    
    pwmQueue = xQueueCreate(4, sizeof(PWM_Message_t));  
    updateEZI2CSemaphore = xSemaphoreCreateBinary();
    uartSemaphore = xSemaphoreCreateBinary();
    
    pwmEventGroup = xEventGroupCreate();
  
    Cy_GPIO_Write(BLUE_PORT,BLUE_NUM,1);
    Cy_GPIO_Write(RED_PORT,RED_NUM,1);
    Cy_GPIO_Write(GREEN_PORT,GREEN_NUM,1);
    
    xTaskCreate( uartTask, "UART Task",400,0,2,0);
    xTaskCreate( pwmTask, "PWM Task",400,0,2,0);
    xTaskCreate( ezi2cTask, "EZI2C Task",400,0,2,0);
    xTaskCreate( capsenseTask, "Capsense Task",(2*1024),0,2,0);
    xTaskCreate( bleTask, "BLE Task",(4*1024),0,3,&bleTaskHandle);
    
    vTaskStartScheduler(); // this function never returns

    while(1); // get rid of the compiler warning
}