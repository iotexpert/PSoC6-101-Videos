
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

// These 
#define SBP_NAME "P6LED"
#define SBP_NAME_OFFET 5
#define SBP_ATT_LEN 28
#define SBP_SERVICE_OFFSET 12


#define BRIGHT_INCREMENT 10
#define BRIGHT_MAX 100

void writeLed(uint8_t brightness)
{
    
    if(Cy_BLE_GetConnectionState( cy_ble_connHandle[0]) != CY_BLE_CONN_STATE_CLIENT_DISCOVERED)
    {
        printf("Not connected\n");
        return;
    }

    printf("Brightness = %d\r\n",brightness);
    
    cy_stc_ble_gattc_write_req_t myVal;
    myVal.handleValPair.attrHandle = cy_ble_customCServ [CY_BLE_CUSTOMC_LED_SERVICE_INDEX].customServChar[CY_BLE_CUSTOMC_LED_GREEN_CHAR_INDEX].customServCharHandle[0];
    myVal.handleValPair.value.val = &brightness;
    myVal.handleValPair.value.actualLen = 1;
    myVal.handleValPair.value.len = 1;
    myVal.connHandle = cy_ble_connHandle[0];
    
    if(Cy_BLE_GATTC_WriteCharacteristicValue( &myVal ) != CY_BLE_SUCCESS)
        printf("gatc write error\n\r");
}

void customEventHandler(uint32_t event, void *eventParameter)
{   
    cy_stc_ble_gapc_adv_report_param_t  *scanProgressParam;
    
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            printf("Starting Scan\r\n");
            Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST,0);  
        
        break;
        case CY_BLE_EVT_GAPC_SCAN_PROGRESS_RESULT:
            // Print Out Information about the Device that was found
            printf("Device ");
            scanProgressParam = (cy_stc_ble_gapc_adv_report_param_t  *)eventParameter;
            printf("BD Addr = ");
            for(unsigned int i=0;i<CY_BLE_BD_ADDR_SIZE;i++)
                printf("%2X",scanProgressParam->peerBdAddr[i]);
            
            printf(" Length = %d\r\n",scanProgressParam->dataLen);

            // If it is the P6LED then make a connection and stop scanning
            if(scanProgressParam->dataLen == SBP_ATT_LEN 
               && strncmp(SBP_NAME,(const char *)&scanProgressParam->data[SBP_NAME_OFFET],strlen(SBP_NAME)) == 0 
               && memcmp(cy_ble_customCServ [CY_BLE_CUSTOMC_LED_SERVICE_INDEX].uuid,&scanProgressParam->data[SBP_SERVICE_OFFSET],16) == 0)
            {
                printf("Found %s\r\n",SBP_NAME);
                // make a connection
                cy_stc_ble_bd_addr_t connectAddr;
                memcpy(&connectAddr.bdAddr[0], &scanProgressParam->peerBdAddr[0] , CY_BLE_BD_ADDR_SIZE);
                connectAddr.type = scanProgressParam->peerAddrType;
                Cy_BLE_GAPC_ConnectDevice(&connectAddr,0);
                Cy_BLE_GAPC_StopScan();
            }
        break;

        case CY_BLE_EVT_GATT_CONNECT_IND:
            printf("Made a connection starting service discovery\r\n");
            Cy_BLE_GATTC_StartDiscovery(cy_ble_connHandle[0]);
        break;
        
        case CY_BLE_EVT_GATTC_DISCOVERY_COMPLETE:
            printf("Discovery Complete\r\n" );
        break;
            
        case CY_BLE_EVT_GATTC_ERROR_RSP:
            printf("GattC Error Response\r\n");
        break;
            
        case CY_BLE_EVT_GATTC_WRITE_RSP:
            printf("GATC Write Succeeded\r\n");
        break;
         
        default:
        break;
    }
}

void bleTask(void *arg)
{
    (void)arg;
    
    Cy_BLE_Start(customEventHandler);
    
    char c;
    uint8_t brightness = 0;
    for(;;)
    {
        Cy_BLE_ProcessEvents();
        if(Cy_SCB_UART_GetNumInRxFifo(UART_1_HW))
        {
            c = getchar();
            switch(c)
            {
                case '+': // Increase the brightness and then send it via BLE
                    brightness = brightness + BRIGHT_INCREMENT > BRIGHT_MAX? BRIGHT_MAX : brightness + BRIGHT_INCREMENT;
                    writeLed(brightness);
                break;
                case '-': // Decrease the brightness and then send it via BLE
                    brightness = (brightness > BRIGHT_INCREMENT ? brightness - BRIGHT_INCREMENT : 0); 
                    writeLed(brightness);
                break;    
            }
        }
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    UART_1_Start();
    setvbuf(stdin,0,_IONBF,0);
    
    printf("Started\n");
    xTaskCreate(bleTask,"bleTask",8*1024,0,1,0);
    vTaskStartScheduler();
 
}
