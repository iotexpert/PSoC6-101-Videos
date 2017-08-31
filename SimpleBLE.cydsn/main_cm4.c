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
cy_stc_ble_conn_handle_t connectionHandle;

void customEventHandler(uint32_t event, void *eventParameter)
{
    //Cy_GPIO_Write(GREEN_PORT,GREEN_NUM,0);
    /* Local variable to store the data received as part of the write request
       events */
    cy_stc_ble_gatts_write_cmd_req_param_t   *writeReqParameter;
    
    /* Take an action based on the current event */
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
            
        Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            
            break;

        /* ~~~~~~~~~~~~~~~~~~~~~~GAP EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
            
        /* If the current BLE state is Disconnected, then the Advertisement
           Start-Stop event implies that advertisement has stopped */
        case CY_BLE_EVT_GAPP_ADVERTISEMENT_START_STOP:
            
            /* Check if the advertisement has stopped */
            if (Cy_BLE_GetState() == CY_BLE_STATE_STOPPED)
            {
                Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            }
            break;
        
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST,
                       CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            break;

        /* ~~~~~~~~~~~~~~~~~~~~~~GATT EVENTS ~~~~~~~~~~~~~~~~~~~~~~~~~~*/
        
        /* This event is received when device is connected over GATT level */    
        case CY_BLE_EVT_GATT_CONNECT_IND:
            
            /* Update attribute handle on GATT Connection*/
            connectionHandle = *(cy_stc_ble_conn_handle_t *) eventParameter;

            /* This flag is used by the application to check the connection
               status */
  //          deviceConnected = true;
            break;
        
        /* This event is received when device is disconnected */
        case CY_BLE_EVT_GATT_DISCONNECT_IND:
           
            /* Update deviceConnected flag*/
    //        deviceConnected = false;
            
            break;
        
        /* This event is received when Central device sends a Write command
           on an Attribute */
        case CY_BLE_EVT_GATTS_WRITE_REQ:
      
            /* Read the write request parameter */
            writeReqParameter = (cy_stc_ble_gatts_write_cmd_req_param_t *) 
                                eventParameter;
     #if 0 
        
            /* When this event is triggered, the peripheral has received a 
               write command on the custom  characteristic. Check if command
               fits any of the custom attributes and update the flag for
               sending notifications by the respective service */
            if (CY_BLE_CAPSENSE_SLIDER_CAPSENSE_SLIDER_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE
                == writeReqParameter->handleValPair.attrHandle)
            {
                handleWriteRequestforSlider(writeReqParameter);
            }
            if (CY_BLE_CAPSENSE_BUTTON_CAPSENSE_BUTTON_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE
               == writeReqParameter->handleValPair.attrHandle)
            {
                handleWriteRequestforButtons(writeReqParameter);
            }
            if (CY_BLE_RGB_LED_RGB_LED_CONTROL_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE
                == writeReqParameter->handleValPair.attrHandle)
            {
                handleWriteRequestforRGB(writeReqParameter);
            }
            /* Check if the returned handle is matching to RGB LED Control Write
               Attribute and extract the RGB data*/
            if (CY_BLE_RGB_LED_RGB_LED_CONTROL_CHAR_HANDLE 
                == writeReqParameter->handleValPair.attrHandle)
            {
                /* Extract the Write value sent by the Client for RGB LED Color 
                   characteristic */
                RGBData[RED_INDEX] = 
                        writeReqParameter->handleValPair.value.val[RED_INDEX];
                RGBData[GREEN_INDEX] = 
                        writeReqParameter->handleValPair.value.val[GREEN_INDEX];
                RGBData[BLUE_INDEX] = 
                        writeReqParameter->handleValPair.value.val[BLUE_INDEX];
                RGBData[INTENSITY_INDEX] = 
                        writeReqParameter->handleValPair.value.val[INTENSITY_INDEX];

                /* Update the the attribute for RGB LED read characteristics and
                   set the color of the LED per the received value */
                updateRGB();
            }
            /* Send the response to the write request received. */
            #endif
    
            if(CY_BLE_MOTOR_M1_CHAR_HANDLE == writeReqParameter->handleValPair.attrHandle)
            {
                Cy_TCPWM_PWM_SetCompare0(PWM_1_HW,PWM_1_CNT_NUM,  writeReqParameter->handleValPair.value.val[0]);
            }
            
            Cy_BLE_GATTS_WriteRsp(connectionHandle);
            break;
        
        /* This event is generated when the internal stack buffer is full and no
           more data can be accepted or the stack has buffer available and can 
           accept data. This event is used by application to prevent pushing lot
           of data to the BLE stack. */
    
        case CY_BLE_EVT_STACK_BUSY_STATUS:
            
            /* Extract the present stack status */
     //       busyStatus = *(uint8_t *) eventParameter;
            break;
        
        /* Do nothing for all other events */
        default:
            break;
    }
}

void bleTask(void *arg)
{
    (void)arg;
    
    Cy_BLE_Start(customEventHandler);
    PWM_1_Start();
    for(;;)
    {
         Cy_BLE_ProcessEvents();
    }
}

int main(void)
{
       __enable_irq(); /* Enable global interrupts. */

    xTaskCreate(bleTask,"bleTask",1024,0,1,0);
    vTaskStartScheduler();
 
}

/* [] END OF FILE */
