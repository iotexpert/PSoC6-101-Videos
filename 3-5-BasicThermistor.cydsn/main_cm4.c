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
#include <math.h>

/* Reference resistor in series with the thermistor is of 10 KOhm */
#define R_REFERENCE         (float)(10000)

/* Beta constant of this thermistor is 3380 Kelvin. See the thermistor
   (NCP18XH103F03RB) data sheet for more details. */
#define B_CONSTANT          (float)(3380)

/* Resistance of the thermistor is 10K at 25 degrees C (from data sheet)
   Therefore R0 = 10000 Ohm, and T0 = 298.15 Kelvin, which gives
   R_INFINITY = R0 e^(-B_CONSTANT / T0) = 0.1192855 */
#define R_INFINITY          (float)(0.1192855)

/* Zero Kelvin in degree C */
#define ABSOLUTE_ZERO       (float)(-273.15)

float GetTemperature(void)
{   
    float v1,v2;
    /* Variables used to store ADC counts, thermistor resistance and
       the temperature */
    int16_t countThermistor, countReference;
    float rThermistor, temperature;

    ADC_IsEndConversion(CY_SAR_WAIT_FOR_RESULT);
    
    /* Read the ADC count values */
    countReference  = ADC_GetResult16(0);
    countThermistor = ADC_GetResult16(1);
   
    v1 = Cy_SAR_CountsTo_Volts(ADC_SAR__HW,0,countReference);
    v2 = Cy_SAR_CountsTo_Volts(ADC_SAR__HW,1,countThermistor);
    
    /* Calculate the thermistor resistance and the corresponding temperature */
    rThermistor = (R_REFERENCE*countThermistor)/countReference;    
    temperature = (B_CONSTANT/(logf(rThermistor/R_INFINITY)))+ABSOLUTE_ZERO;

     printf("V1 = %fv V2=%fv Vtot =%fv T=%fC T=%fF\r\n",v1,v2,v1+v2,temperature,9.0/5.0*temperature + 32.0);

    
    /* Return the temperature value */
    return temperature;
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */

    ADC_Start();
    ADC_StartConvert();
    UART_1_Start();
    Opamp_1_Start();
    
    float temperature;
    Cy_GPIO_Write(A0_PORT,A0_NUM,1);
    Cy_GPIO_Write(A3_PORT,A3_NUM,0);
    
    setvbuf(stdin,0,_IONBF,0);
    char c;
    
    for(;;)
    {
        if(Cy_SCB_UART_GetNumInRxFifo(UART_1_HW))
        {
            c = Cy_SCB_UART_Get(UART_1_HW);
            temperature = GetTemperature();
            
        }
        
    }
}

/* [] END OF FILE */
