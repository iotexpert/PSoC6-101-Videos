#include "project.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "bmi160.h"
#include "semphr.h"
#include "event_groups.h"

TaskHandle_t bleTaskHandle;
SemaphoreHandle_t bleSemaphore;
EventGroupHandle_t systemInputMode;

#define MODE_CAPSENSE    (1<<0)


#define P6ROBOT_NAME "P6Robot"
#define P6ROBOT_NAME_OFFET 5
#define P6ROBOT_ATT_LEN 30
#define P6ROBOT_SERVICE_OFFSET 14

#define THRESHOLD 1

void writeMotor(uint8_t motor, uint8_t pos)
{
    
    static int prev_m1 = 500;
    static int prev_m2 = 500;
    
    if(Cy_BLE_GetConnectionState( cy_ble_connHandle[0]) != CY_BLE_CONN_STATE_CLIENT_DISCOVERED)
    {
        return;
    }
    
    if(Cy_BLE_GATT_GetBusyStatus(cy_ble_connHandle[0].attId) == CY_BLE_STACK_STATE_BUSY)
        return;
   
    
    cy_stc_ble_gattc_write_req_t myVal;
    if(motor == 1)
    {
        if (abs(prev_m1 - pos) < THRESHOLD)
            return;
    
        prev_m1 = pos;
        myVal.handleValPair.attrHandle = cy_ble_customCServ [CY_BLE_CUSTOMC_MOTOR_SERVICE_INDEX].customServChar[CY_BLE_CUSTOMC_MOTOR_M1_CHAR_INDEX].customServCharHandle[0];
    }
    else
    {
        if (abs(prev_m2 - pos) < THRESHOLD)
            return;
        prev_m2 = pos;
        myVal.handleValPair.attrHandle = cy_ble_customCServ [CY_BLE_CUSTOMC_MOTOR_SERVICE_INDEX].customServChar[CY_BLE_CUSTOMC_MOTOR_M2_CHAR_INDEX].customServCharHandle[0];
    }
    
    printf("Motor %d Val = %d\r\n",motor,pos);
    
    myVal.handleValPair.value.val = &pos;
    myVal.handleValPair.value.len = 1;
    myVal.connHandle = cy_ble_connHandle[0];
    
    Cy_BLE_GATTC_WriteCharacteristicValue( &myVal );
      
}

typedef struct  {
    char *name;
    int name_len;
    uint8_t *serviceUUID;
    uint8 servUUID_len;
} advInfo_t;

advInfo_t currentAdvInfo;

int findAdvInfo(uint8_t *adv,uint8_t len)
{
    memset(&currentAdvInfo,0,sizeof(currentAdvInfo));
    
    for(uint8_t i=0;i<len;)
    {
        switch(adv[i+1])
        {
            case 7: // Service UUID
                currentAdvInfo.serviceUUID = &adv[i+2];
                currentAdvInfo.servUUID_len = adv[i]-1;
            break;
            
            case 9: // Name
                currentAdvInfo.name = (char *)&adv[i+2];
                currentAdvInfo.name_len = adv[i]-1;
            break;
        }
        i = i + adv[i]+1;
    }
    return (currentAdvInfo.name_len > 0 && currentAdvInfo.serviceUUID > 0);
}

void capsenseTask(void *arg)
{
    (void)arg;
    
    printf("Started CapSense Task\r\n");
    CapSense_Start();
    CapSense_ScanAllWidgets();
    int currentMotor = 1;
    
    for(;;)
    {
        xEventGroupWaitBits(systemInputMode,MODE_CAPSENSE,pdFALSE,pdTRUE,portMAX_DELAY);
        if(!CapSense_IsBusy())
        {
            CapSense_ProcessAllWidgets();
            int pos;
            pos=CapSense_GetCentroidPos(CapSense_LINEARSLIDER0_WDGT_ID);
            if(pos<0xFFFF && (xEventGroupGetBits(systemInputMode) & MODE_CAPSENSE ) )
            {
                printf("CAP Motor=%d Pos=%d\r\n",currentMotor,pos);
                writeMotor(currentMotor,pos);
            }   
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
    static cy_stc_ble_bd_addr_t connectAddr;
                
    switch (event)
    {
        /* This event is received when the BLE stack is Started */
        case CY_BLE_EVT_STACK_ON:
            printf("Stack On\r\n");
            break;
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
            Cy_GPIO_Write(LED8_PORT,LED8_0_NUM,1);
            printf("Started Scanning\r\n");
            Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST,0);  
        
        break;
        case CY_BLE_EVT_GAPC_SCAN_PROGRESS_RESULT:
            // Print Out Information about the Device that was found
            scanProgressParam = (cy_stc_ble_gapc_adv_report_param_t  *)eventParameter;
            findAdvInfo(scanProgressParam->data,scanProgressParam->dataLen);
            printf("Device ");
            if(currentAdvInfo.name_len)
            {
                printf(" Name=");
                for(int i=0;i<currentAdvInfo.name_len;i++)
                {
                    putchar(currentAdvInfo.name[i]);
                }
            }
            
            if(currentAdvInfo.servUUID_len)
            {
                printf(" UUID=");
                for(int i=0;i<currentAdvInfo.servUUID_len;i++)
                {
                    printf("%02X",currentAdvInfo.serviceUUID[i]);
                }
            }
            
            printf("\r\n");
            
            if(currentAdvInfo.servUUID_len>0 &&
                memcmp(cy_ble_customCServ [CY_BLE_CUSTOMC_MOTOR_SERVICE_INDEX].uuid,currentAdvInfo.serviceUUID,16) == 0)
            {
                   
                
 #if 0           
            // If it is the P6LED then make a connection and stop scanning
            if(scanProgressParam->dataLen == P6ROBOT_ATT_LEN 
               && strncmp(P6ROBOT_NAME,(const char *)&scanProgressParam->data[P6ROBOT_NAME_OFFET],strlen(P6ROBOT_NAME)) == 0 
               && memcmp(cy_ble_customCServ [CY_BLE_CUSTOMC_MOTOR_SERVICE_INDEX].uuid,&scanProgressParam->data[P6ROBOT_SERVICE_OFFSET],16) == 0)
            {
#endif
                printf("Found %s\r\n",P6ROBOT_NAME);
                
                memcpy(&connectAddr.bdAddr[0], &scanProgressParam->peerBdAddr[0] , CY_BLE_BD_ADDR_SIZE);
                connectAddr.type = scanProgressParam->peerAddrType;
                
                Cy_BLE_GAPC_StopScan();
            }
        break;
            
        case CY_BLE_EVT_GAPC_SCAN_START_STOP:
            if(Cy_BLE_GetScanState() == CY_BLE_SCAN_STATE_STOPPED)
            {
                printf("Started connection\n");
                Cy_BLE_GAPC_ConnectDevice(&connectAddr,0);
            }
            else
            {
                printf("Started scan\n");
            }
        break;
            

        case CY_BLE_EVT_GATT_CONNECT_IND:
            Cy_BLE_GATTC_StartDiscovery(cy_ble_connHandle[0]);
            printf("Starting Discovery of Services\r\n");
        break;
            
        case CY_BLE_EVT_GATTC_DISCOVERY_COMPLETE:
            Cy_GPIO_Write(LED8_PORT,LED8_0_NUM,0);
            printf("Discovery Complete\r\n" );
        break;
            
            case CY_BLE_EVT_GATTC_WRITE_RSP:
                printf("Sucessful write\r\n");
            break;
              
        default:
        break;
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
    printf("Started BLE Task\r\n");
    bleSemaphore = xSemaphoreCreateCounting(0xFFFFFFFF,0);

    
    Cy_BLE_Start(customEventHandler);
    Cy_BLE_RegisterAppHostCallback(bleInterruptNotify);

    while(Cy_BLE_GetState() != CY_BLE_STATE_ON)
    {
        Cy_BLE_ProcessEvents();
    }
 
    
    Cy_BLE_GAPC_StartScan(CY_BLE_SCANNING_FAST,0);  
   
    for(;;)
    {
        xSemaphoreTake(bleSemaphore,portMAX_DELAY);
        Cy_BLE_ProcessEvents();         
    }
}


static struct bmi160_dev bmi160Dev;

int8_t BMI160BurstWrite(uint8_t dev_addr, uint8_t reg_addr,uint8_t *data, uint16_t len)
{
    
    Cy_SCB_I2C_MasterSendStart(I2C_1_HW,dev_addr,CY_SCB_I2C_WRITE_XFER,0,&I2C_1_context);
    Cy_SCB_I2C_MasterWriteByte(I2C_1_HW,reg_addr,0,&I2C_1_context);
    for(int i = 0;i<len; i++)
    { 
        Cy_SCB_I2C_MasterWriteByte(I2C_1_HW,data[i],0,&I2C_1_context);
    }
    
    Cy_SCB_I2C_MasterSendStop(I2C_1_HW,0,&I2C_1_context);
    
    return 0;
}

// This function supports the BMP180 library and read I2C Registers
int8_t BMI160BurstRead(uint8_t dev_addr, uint8_t reg_addr,uint8_t *data, uint16_t len)
{
    
    Cy_SCB_I2C_MasterSendStart(I2C_1_HW,dev_addr,CY_SCB_I2C_WRITE_XFER,0,&I2C_1_context);
    Cy_SCB_I2C_MasterWriteByte(I2C_1_HW,reg_addr,0,&I2C_1_context);
    Cy_SCB_I2C_MasterSendReStart(I2C_1_HW,dev_addr,CY_SCB_I2C_READ_XFER,0,&I2C_1_context);
    for(int i = 0;i<len-1; i++)
    {
        Cy_SCB_I2C_MasterReadByte(I2C_1_HW,CY_SCB_I2C_ACK,&data[i],0,&I2C_1_context);
    }
    Cy_SCB_I2C_MasterReadByte(I2C_1_HW,CY_SCB_I2C_NAK,&data[len-1],0,&I2C_1_context);
    
    Cy_SCB_I2C_MasterSendStop(I2C_1_HW,0,&I2C_1_context);
    
    
    return 0;
}


static void sensorsDeviceInit(void)
{

  int8_t rslt;
  vTaskDelay(500); // guess

  /* BMI160 */
  // assign bus read function
  bmi160Dev.read = (bmi160_com_fptr_t)BMI160BurstRead;
  // assign bus write function
  bmi160Dev.write = (bmi160_com_fptr_t)BMI160BurstWrite;
  // assign delay function
  bmi160Dev.delay_ms = (bmi160_delay_fptr_t)vTaskDelay;
  bmi160Dev.id = BMI160_I2C_ADDR;  // I2C device address

  rslt = bmi160_init(&bmi160Dev); // initialize the device
  if (rslt == 0)
    {
      printf("BMI160 I2C connection [OK].\n");
      /* Select the Output data rate, range of Gyroscope sensor
       * ~92Hz BW by OSR4 @ODR=800Hz */
      bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
      bmi160Dev.gyro_cfg.range = BMI160_GYRO_RANGE_125_DPS;
      bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_OSR4_MODE;
      bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

      bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
      bmi160Dev.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
      bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_OSR4_AVG1;
      bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

      /* Set the sensor configuration */
      rslt |= bmi160_set_sens_conf(&bmi160Dev);
      bmi160Dev.delay_ms(50);
    }
  else
    {
      printf("BMI160 I2C connection [FAIL].\n");
    }
}
#define MAXACCEL 8000
void motionTask(void *arg)
{
    (void)arg;
    printf("Motion Task Started\r\n");
    I2C_1_Start();
    sensorsDeviceInit();
    TickType_t lastMovement;
    
    lastMovement = 0;
    
    struct bmi160_sensor_data acc;
    
    float m1,m2;
    float iirm1 = 50.0;
    float iirm2 = 50.0;
    xEventGroupSetBits(systemInputMode,MODE_CAPSENSE);
    
    while(1)
    {
        
        bmi160_get_sensor_data(BMI160_ACCEL_ONLY, &acc, NULL, &bmi160Dev);
        
        acc.x = (acc.x>MAXACCEL)?MAXACCEL:acc.x;
        acc.x = (acc.x<-MAXACCEL)?-MAXACCEL:acc.x;

        acc.y = (acc.y>MAXACCEL)?MAXACCEL:acc.y;
        acc.y = (acc.y<-MAXACCEL)?-MAXACCEL:acc.y;
        
        
  
        m1 = acos((double)(acc.y)  / (double)MAXACCEL)*360/(2*3.14*1.8);
        m2 = acos((double)acc.x  / (double)MAXACCEL)*360/(2*3.14*1.8);
        // IIR Filter to get rid of noise
        iirm1 = m1 / 8.0 + iirm1*7.0/8.0;
        iirm2 = m2 / 8.0 + iirm2*7.0/8.0;
        
        //printf("x=%4d y=%4d z=%4d m1=%3.0f m2=%2.0f IIR m1=%3.0f m2=%2.0f\r\n",acc.x,acc.y,acc.z,m1,m2,iirm1,iirm2);
 
        if( fabs(m1-50.0)>3.0 || fabs(m2-50.0) > 3.0)
        {
            lastMovement = xTaskGetTickCount();
        }
        
        // if it has been more than a second then turn on capsense and LED
        if( (xTaskGetTickCount() - lastMovement) > 1000)
        {
            xEventGroupSetBits(systemInputMode,MODE_CAPSENSE);
            Cy_GPIO_Write(LED9_PORT,LED9_NUM,0);
        }
        else  
        {
            xEventGroupClearBits(systemInputMode,MODE_CAPSENSE);
            Cy_GPIO_Write(LED9_PORT,LED9_NUM,1);
        }
      
        
        if(xEventGroupGetBits(systemInputMode)== 0)
        {
            writeMotor(1,iirm1);
            writeMotor(2,iirm2);
        }
    }
}


int main(void)
{
    __enable_irq(); /* Enable global interrupts. */
    UART_1_Start();
    printf("Started Project\r\n");
    systemInputMode = xEventGroupCreate();
    
    xTaskCreate( capsenseTask, "CapSense Task",400,0,1,0);
    xTaskCreate(bleTask,"bleTask",4*1024,0,2,&bleTaskHandle);
    xTaskCreate( motionTask, "Motion Task",400,0,1,0);
    vTaskStartScheduler();
 
}

