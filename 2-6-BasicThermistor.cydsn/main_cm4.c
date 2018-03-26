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
#include "Thermistor.h"

float GetTemperature(void)
{   
    float v1,v2;
    /* Variables used to store ADC counts, thermistor resistance and
       the temperature */
    int16_t countThermistor, countReference;


    ADC_IsEndConversion(CY_SAR_WAIT_FOR_RESULT);
    
    /* Read the ADC count values */
    countReference  = ADC_GetResult16(0);
    countThermistor = ADC_GetResult16(1);
   
    v1 = Cy_SAR_CountsTo_Volts(ADC_SAR__HW,0,countReference);
    v2 = Cy_SAR_CountsTo_Volts(ADC_SAR__HW,1,countThermistor);
 
    uint32 resT = Thermistor_GetResistance(countReference, countThermistor);
    float temperature = (float)Thermistor_GetTemperature(resT) / 100.0 ;
 
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
    
    printf("Started\r\n");
    float temperature;
    Cy_GPIO_Write(A0_PORT,A0_NUM,1);
    Cy_GPIO_Write(A3_PORT,A3_NUM,0);
    
    setvbuf(stdin,0,_IONBF,0);
    char c;
    
    for(;;)
    {
        GetTemperature();
        CyDelay(200);
    }
}

/* [] END OF FILE */
