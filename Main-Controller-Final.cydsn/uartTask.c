#include <project.h>
#include "FreeRTOS.h"
#include "global.h"
#include "semphr.h"
#include "pwmTask.h"
#include <stdio.h>

static char buff[128];

// This ISR is called by the command line UART when a key is pressed
void UART_Isr()
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
    
    UART_PutString("Started UART\r\n");
    
    //(void) Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);

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
                    sprintf(buff,"M1=%d M2=%d\n",getMotorPercent(M1),getMotorPercent(M2));
                    Cy_SCB_UART_PutString(UART_HW,buff);
                      
                break;
                    
                case 'p': // Make Motor 1 +10%
                    myMessage.motor = M1; myMessage.percent=-1; myMessage.percentChange=10;
                    xQueueSend(pwmQueue,&myMessage,0);
                    
                break;
            
                case 'o':
                    myMessage.motor = M1; myMessage.percent=-1; myMessage.percentChange=-10;
                    xQueueSend(pwmQueue,&myMessage,0);
                break;
                    
                case 'l': // Make Motor 2 +10%
                    myMessage.motor = M2; myMessage.percent=-1; myMessage.percentChange=10;
                    xQueueSend(pwmQueue,&myMessage,0);
                    
                break;
            
                case 'k':
                    myMessage.motor = M2; myMessage.percent=-1; myMessage.percentChange=-10;
                    xQueueSend(pwmQueue,&myMessage,0);
                break;
        
                case '5':
                    myMessage.motor = M1; myMessage.percent=50; 
                    xQueueSend(pwmQueue,&myMessage,10);
                    myMessage.motor = M2; myMessage.percent=50; 
                    xQueueSend(pwmQueue,&myMessage,10);
                
                break;
                    
                case '?':
                    Cy_SCB_UART_PutString(UART_HW,"o\tM1 -10%\n");
                    Cy_SCB_UART_PutString(UART_HW,"p\tM1 +10%\n");
                    Cy_SCB_UART_PutString(UART_HW,"k\tM2 -10%\n");
                    Cy_SCB_UART_PutString(UART_HW,"l\tM2 +10%\n");
                    Cy_SCB_UART_PutString(UART_HW,"s\tStatus\n");
                    Cy_SCB_UART_PutString(UART_HW,"5\tM1=50% M2=50%\n");
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