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
#include <ctype.h>

int main(void)
{
    char c;
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    UART_Start();

    
    for(;;)
    {
        c = Cy_SCB_UART_Get(UART_HW);
        if(isprint(c))
            Cy_SCB_UART_Put(UART_HW,c);
        
    }
}
