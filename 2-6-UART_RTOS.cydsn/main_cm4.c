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
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

SemaphoreHandle_t uartSemaphore;

// This ISR is called by the command line UART when a key is pressed
void UART_Isr()
{
    // Disable the interrupt
    // read the interrupt mask
    uint32_t mask = Cy_SCB_GetRxInterruptMask(UART_HW);
    mask &= ~CY_SCB_RX_INTR_NOT_EMPTY;
    Cy_SCB_SetRxInterruptMask(UART_HW,mask);
    
    Cy_SCB_ClearRxInterrupt(UART_HW, CY_SCB_RX_INTR_NOT_EMPTY);
    NVIC_ClearPendingIRQ(SysInt_1_cfg.intrSrc);

    // If the semaphore causes a task switch you should yield to
    // that task
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(uartSemaphore,&xHigherPriorityTaskWoken); // Tell the UART thread to process the RX FIFO
    if(xHigherPriorityTaskWoken != pdFALSE)
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void uartTask(void *arg)
{
    char c;

    (void)arg;
    
    UART_Start();
    // Turn off input and output buffering
    setvbuf( stdin, NULL, _IONBF, 0 ); 
    setvbuf( stdout, NULL, _IONBF, 0 ); 
    
    printf("Started UART Task\r\n");
    
    uartSemaphore = xSemaphoreCreateBinary();
    
    /* Hook interrupt service routine and enable interrupt */
    Cy_SysInt_Init(&SysInt_1_cfg, &UART_Isr);
    NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);
       
    while(1)
    {
        xSemaphoreTake(uartSemaphore,portMAX_DELAY);

        while(Cy_SCB_UART_GetNumInRxFifo(UART_HW))
        {
            c = getchar();
            putchar(c);
        }
        // Once you have processed the RX FIFO you should
        // reenable the interrupt
        uint32_t mask = Cy_SCB_GetRxInterruptMask(UART_HW);
        mask |= CY_SCB_RX_INTR_NOT_EMPTY;
        Cy_SCB_SetRxInterruptMask(UART_HW,mask);
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    xTaskCreate( uartTask, "UART Task",400,0,1,0);
    vTaskStartScheduler();
    for(;;)
    {
        /* Place your application code here. */
    }
}

/* [] END OF FILE */
