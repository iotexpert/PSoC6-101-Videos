

#include "project.h"

void ledTask(void *arg)
{
    (void)arg;
    
    while(1)
    {
        Cy_GPIO_Inv(RED_PORT,RED_NUM);
        vTaskDelay(500);
        
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    vTraceEnable(TRC_START);

    xTaskCreate(ledTask,"LED Task",configMINIMAL_STACK_SIZE,0,1,0);
    vTaskStartScheduler();
    

    for(;;)
    {
    }
}

/* [] END OF FILE */
