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

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    
    UART_Start();
    
   setvbuf( stdin, NULL, _IONBF, 0 );

   printf("Started Retarget UART Example\r\n");
    
    char c;

    for(;;)
    {
        c = getchar();
        if(c)
        {
            printf("%c",c);
        }
        
    }
}

/* [] END OF FILE */
