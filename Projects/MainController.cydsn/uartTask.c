#include <project.h>
#include <stdio.h>
#include "pwmTask.h"
#include "global.h"

SemaphoreHandle_t uartSemaphore;

// This ISR is called by the command line UART when a key is pressed
static void UART_Isr()
{
    // Disable the interrupt
    // read the interrupt mask
    uint32_t mask = Cy_SCB_GetRxInterruptMask(UART_HW);
    mask &= ~CY_SCB_RX_INTR_NOT_EMPTY;
    Cy_SCB_SetRxInterruptMask(UART_HW,mask);
    
    Cy_SCB_ClearRxInterrupt(UART_HW, CY_SCB_RX_INTR_NOT_EMPTY);
    NVIC_ClearPendingIRQ(SysInt_UART_cfg.intrSrc);

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
    (void)arg;
    PWM_Message_t myMessage;
    UART_Start();
    uartSemaphore = xSemaphoreCreateBinary();    

    
    printf("Started UART\r\n");
   
    /* Hook interrupt service routine and enable interrupt */
    Cy_SysInt_Init(&SysInt_UART_cfg, &UART_Isr);
    NVIC_EnableIRQ(SysInt_UART_cfg.intrSrc);
    Cy_SCB_UART_Enable(UART_HW);

    char c;
    
    while(1)
    {
        xSemaphoreTake(uartSemaphore,portMAX_DELAY);

        while(Cy_SCB_UART_GetNumInRxFifo(UART_HW))
        {
            c= Cy_SCB_UART_Get(UART_HW);
            switch(c)
            {
                case 'g':
                    Cy_GPIO_Write(GREEN_PORT,GREEN_NUM,!Cy_GPIO_ReadOut(GREEN_PORT,GREEN_NUM));
                break;
                case 'b':
                    Cy_GPIO_Write(BLUE_PORT,BLUE_NUM,!Cy_GPIO_ReadOut(BLUE_PORT,BLUE_NUM));
                break;
                  
                case 's':
                    printf("M1=%d M2=%d\n",getMotorPercent(M1),getMotorPercent(M2));
                      
                break;
                    
                case 'p': // Make Motor 1 +10%
                    myMessage.motor = M1; myMessage.changeType = POS_RELATIVE; myMessage.percent=10;
                    xQueueSend(pwmQueue,&myMessage,0);
                    
                break;
            
                case 'o':
                    myMessage.motor = M1; myMessage.changeType = POS_RELATIVE; myMessage.percent=-10;
                    xQueueSend(pwmQueue,&myMessage,0);
                break;
                    
                case 'l': // Make Motor 2 +10%
                    myMessage.motor = M2; myMessage.changeType = POS_RELATIVE; myMessage.percent =10;
                    xQueueSend(pwmQueue,&myMessage,0);
                    
                break;
            
                case 'k':
                    myMessage.motor = M2; myMessage.changeType = POS_RELATIVE; myMessage.percent=-10;
                    xQueueSend(pwmQueue,&myMessage,0);
                break;
        
                case '5':
                    myMessage.motor = M1; myMessage.percent=50; 
                    xQueueSend(pwmQueue,&myMessage,10);
                    myMessage.motor = M2; myMessage.percent=50; 
                    xQueueSend(pwmQueue,&myMessage,10);
                
                break;
                    
                case '?':
                    printf("o\tM1 -10%%\n");
                    printf("p\tM1 +10%%\n");
                    printf("k\tM2 -10%%\n");
                    printf("l\tM2 +10%%\n");
                    printf("s\tStatus\n");
                    printf("5\tM1=50%% M2=50%%\n");
                break;
            }
        }
        // Once you have processed the RX FIFO you should
        // reenable the interrupt
        uint32_t mask = Cy_SCB_GetRxInterruptMask(UART_HW);
        mask |= CY_SCB_RX_INTR_NOT_EMPTY;
        Cy_SCB_SetRxInterruptMask(UART_HW,mask);
    }
}