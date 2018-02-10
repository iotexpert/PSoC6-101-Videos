#include "project.h"
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"

SemaphoreHandle_t bleSemaphore;
TaskHandle_t bleTaskHandle;
EventGroupHandle_t alertState;

#define ALERT_NO_MASK   1<<0
#define ALERT_MILD_MASK 1<<1
#define ALERT_HIGH_MASK 1<<2

void alertTask(void *arg)
{
    (void)arg;
    printf("Alert Task Started\r\n");
    TickType_t delay=portMAX_DELAY;
    EventBits_t currentBits;
    
    alertState = xEventGroupCreate();
    xEventGroupSetBits(alertState,ALERT_NO_MASK);
    Cy_GPIO_Write(red_PORT,red_NUM,1);
    
    while(1)
    {
        currentBits = xEventGroupWaitBits(alertState,ALERT_HIGH_MASK|ALERT_MILD_MASK|ALERT_NO_MASK,
            pdTRUE,pdFALSE,delay);
        switch(currentBits)
        {
            case ALERT_NO_MASK:
                delay = portMAX_DELAY;
                Cy_GPIO_Write(red_PORT,red_NUM,1);
            break;
            case ALERT_HIGH_MASK:
                delay = portMAX_DELAY;
                Cy_GPIO_Write(red_PORT,red_NUM,0);
            break;
            case 0: // case 0 means timer expired & no bits set.   
            case ALERT_MILD_MASK:
                delay = 500;
                Cy_GPIO_Inv(red_PORT,red_NUM);
            break;
        }   
    }
}

void customEventHandler(uint32_t event, void *eventParameter)
{
    (void)eventParameter; // not used
    
    /* Take an action based on the current event */
    switch (event)
    {
        case CY_BLE_EVT_STACK_ON:
            printf("Stack Started\r\n");
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;

        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("Disconnected\r\n");
            Cy_GPIO_Write(led9_PORT,led9_NUM,1); // Turn the LED9 Off
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
        break;

        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("Connected\r\n");
            Cy_GPIO_Write(led9_PORT,led9_NUM,0); // Turn the LED9 On             
        break;
                
        default:
        break;
    }
}

void iasCallback(uint32_t eventCode, void *eventParam)
{
    (void)eventParam;
    uint8_t alertLevel;
    
    if(eventCode == CY_BLE_EVT_IASS_WRITE_CHAR_CMD)
    {
        /* Read the updated Alert Level value from the GATT database */
        Cy_BLE_IASS_GetCharacteristicValue(CY_BLE_IAS_ALERT_LEVEL, 
            sizeof(alertLevel), &alertLevel);
        
        switch(alertLevel)
        {
            case CY_BLE_NO_ALERT: // Off
                printf("No alert\n");
                xEventGroupSetBits(alertState,ALERT_NO_MASK);               
            break;
            case CY_BLE_MILD_ALERT: // blink
                printf("Medium alert\n");
                xEventGroupSetBits(alertState,ALERT_MILD_MASK);               
            break;
            case CY_BLE_HIGH_ALERT:        
                printf("High alert\n");
                xEventGroupSetBits(alertState,ALERT_HIGH_MASK);               
            break;
        }   
    }   
}

void bleInterruptNotify()
{
    BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(bleSemaphore, &xHigherPriorityTaskWoken); 
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void bleTask(void *arg)
{
    (void)arg;
    
    printf("BLE Task Started\n");

    bleSemaphore = xSemaphoreCreateCounting(2^32-1,0);
    
    Cy_BLE_Start(customEventHandler);
    Cy_BLE_IPC_RegisterAppHostCallback(bleInterruptNotify);
    
    while(Cy_BLE_GetState() != CY_BLE_STATE_ON)
    {
        Cy_BLE_ProcessEvents();
    }
    
    Cy_BLE_IAS_RegisterAttrCallback (iasCallback);
    for(;;)
    {
        xSemaphoreTake(bleSemaphore,portMAX_DELAY);
        Cy_BLE_ProcessEvents();
    }
}


int main(void)
{
    __enable_irq(); /* Enable global interrupts. */

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    
    UART_1_Start();
    setvbuf( stdin, NULL, _IONBF, 0 ); 
    setvbuf( stdout, NULL, _IONBF, 0 ); 
    printf("System Started\r\n");

    xTaskCreate(bleTask,"bleTask",8*1024,0,2,&bleTaskHandle);
    xTaskCreate(alertTask,"AlertTask",configMINIMAL_STACK_SIZE,0,1,0);
    
    vTaskStartScheduler();
    for(;;)
    {
    }
}
