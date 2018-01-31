
#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>

#define P6ROBOT_NAME "P6Robot"
#define P6ROBOT_NAME_OFFET 5
#define P6ROBOT_ATT_LEN 30
#define P6ROBOT_SERVICE_OFFSET 14


void writeMotor(uint8_t motor, uint8_t pos)
{
    printf("Motor = %d Pos=%d\r\n",motor,pos);
    
    if(Cy_BLE_GetConnectionState( cy_ble_connHandle[0]) != CY_BLE_CONN_STATE_CLIENT_DISCOVERED)
    {
        return;
    }
    
    
    cy_stc_ble_gattc_write_req_t myVal;
    if(motor == 1)
        myVal.handleValPair.attrHandle = cy_ble_customCServ [CY_BLE_CUSTOMC_MOTOR_SERVICE_INDEX].customServChar[CY_BLE_CUSTOMC_MOTOR_M1_CHAR_INDEX].customServCharHandle[0];
    else
        myVal.handleValPair.attrHandle = cy_ble_customCServ [CY_BLE_CUSTOMC_MOTOR_SERVICE_INDEX].customServChar[CY_BLE_CUSTOMC_MOTOR_M2_CHAR_INDEX].customServCharHandle[0];
    
    myVal.handleValPair.value.val = &pos;
    myVal.handleValPair.value.actualLen = 1;
    myVal.handleValPair.value.len = 1;
    myVal.connHandle = cy_ble_connHandle[0];
    
    Cy_BLE_GATTC_WriteCharacteristicValue( &myVal );
      
}

void capsenseTask(void *arg)
{
    (void)arg;
    
    CapSense_Start();
    CapSense_ScanAllWidgets();
    int currentMotor = 1;
    
    for(;;)
    {
        if(!CapSense_IsBusy())
        {
            CapSense_ProcessAllWidgets();
            int pos;
            pos=CapSense_GetCentroidPos(CapSense_LINEARSLIDER0_WDGT_ID);
            if(pos<0xFFFF)
                writeMotor(currentMotor,pos);
                
            if(CapSense_IsWidgetActive(CapSense_BUTTON0_WDGT_ID))
            {
                currentMotor = 1;
            
            }
            if(CapSense_IsWidgetActive(CapSense_BUTTON1_WDGT_ID))
            {
                 currentMotor = 2;
                
            }
                
            CapSense_UpdateAllBaselines();
            CapSense_ScanAllWidgets();
        }
        else
            vTaskDelay(10);
    }
}

void customEventHandler(uint32_t event, void *eventParameter)
{   
    cy_stc_ble_gapc_adv_report_param_t  *scanProgressParam;
    
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST,0);  
        
        break;
        case CY_BLE_EVT_GAPC_SCAN_PROGRESS_RESULT:
            // Print Out Information about the Device that was found
            scanProgressParam = (cy_stc_ble_gapc_adv_report_param_t  *)eventParameter;
            printf("Found Device %d\r\n",scanProgressParam->dataLen);
            
            // If it is the P6LED then make a connection and stop scanning
            if(scanProgressParam->dataLen == P6ROBOT_ATT_LEN 
               && strncmp(P6ROBOT_NAME,(const char *)&scanProgressParam->data[P6ROBOT_NAME_OFFET],strlen(P6ROBOT_NAME)) == 0 
               && memcmp(cy_ble_customCServ [CY_BLE_CUSTOMC_MOTOR_SERVICE_INDEX].uuid,&scanProgressParam->data[P6ROBOT_SERVICE_OFFSET],16) == 0)
            {
                printf("Found %s\r\n",P6ROBOT_NAME);
                // make a connection
                cy_stc_ble_bd_addr_t connectAddr;
                memcpy(&connectAddr.bdAddr[0], &scanProgressParam->peerBdAddr[0] , CY_BLE_BD_ADDR_SIZE);
                connectAddr.type = scanProgressParam->peerAddrType;
                Cy_BLE_GAPC_ConnectDevice(&connectAddr,0);
                Cy_BLE_GAPC_StopScan();
            }
        break;

        case CY_BLE_EVT_GATT_CONNECT_IND:
            Cy_BLE_GATTC_StartDiscovery(cy_ble_connHandle[0]);
            printf("Starting Discovery of Services\r\n");
        break;
            
        case CY_BLE_EVT_GATTC_DISCOVERY_COMPLETE:
            printf("Discovery Complete\r\n" );
        break;
              
        default:
        break;
    }
}

void bleTask(void *arg)
{
    (void)arg;
    printf("Started BLE Task\r\n");
    Cy_BLE_Start(customEventHandler);
    
    for(;;)
    {
        Cy_BLE_ProcessEvents();
        vTaskDelay(5);
    }
}

int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    UART_1_Start();
    printf("Started Project\r\n");
    xTaskCreate( capsenseTask, "CapSense Task",400,0,1,0);
    xTaskCreate(bleTask,"bleTask",4*1024,0,1,0);
    vTaskStartScheduler();
 
}
