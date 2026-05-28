
#include "project.h"
#include "stdio.h"
#include "bmi270.h"
#include "bmi2.h"
#include "BLE.h"

#define LED_ON  1
#define LED_OFF 0

//Battery levels for LOW, MIDDLE, and HIGH
#define UPP_BATT 1.59
#define MIDD_BATT 1.75
#define ON 1
#define OFF 0


static struct bmi2_dev bmi270_dev;
static cy_stc_scb_i2c_master_xfer_config_t register_setting; // this is for i2c commnication 
static uint8_t bmi270_i2c_addr = 0x68;

static uint8 rbuff[8];
static uint8 wbuff[8];
static uint8 burstBuff[1024];

//Our haptic feedback flag, for when BLE turns off and on
int buzzer_flag = 0;

static void WaitForOperation()
{
    uint32_t timeout = 10000; // Emergency exit counter
    while((0 != (SensorBus_MasterGetStatus() & CY_SCB_I2C_MASTER_BUSY)) && (timeout > 0)) 
    {
        timeout--;
        CyDelayUs(1);
    }
}

static void WriteRegister(uint8 reg_addr, uint8 data) {
    wbuff[0] = reg_addr;   //Assign the first element to be the registere you want to write to "reg address"
    wbuff[1] = data;       //Assign the second element to be the value you wish to write to the reg
    
    register_setting.buffer = wbuff;
    register_setting.bufferSize = 2;
    register_setting.xferPending = false;
    
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
    
}


static uint8 ReadRegister(uint8 reg_addr)
{
    wbuff[0] = reg_addr;

    register_setting.buffer = wbuff;
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;

    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();

    register_setting.buffer = rbuff;
    register_setting.bufferSize = 1;
    register_setting.xferPending = false;

    SensorBus_MasterRead(&register_setting);
    WaitForOperation();

    return rbuff[0];
}

static void BMI270_PrepareConfigLoad(void)
{
    // Disable advanced power save: PWR_CONF[0] = adv_power_save = 0
    WriteRegister(0x7C, 0x00);
    CyDelayUs(500);   // Bosch says >= 450 us

    // Prepare config load
    WriteRegister(0x59, 0x00);   // INIT_CTRL = 0x00

    // Start config address at 0
    WriteRegister(0x5B, 0x00);   // INIT_ADDR_0
    WriteRegister(0x5C, 0x00);   // INIT_ADDR_1
}

static void WriteBurst(uint8 reg_addr, const uint8 *data, uint16 length)
{
    uint16 i;

    burstBuff[0] = reg_addr;   // first byte = register address

    for(i = 0; i < length; i++)
    {
        burstBuff[i + 1] = data[i];
    }

    register_setting.buffer = burstBuff;
    register_setting.bufferSize = length + 1;
    register_setting.xferPending = false;

    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();
}

static void BMI270_UploadConfig(const uint8 *config_file, uint16 config_size)
{
    uint16 index = 0;
    uint16 chunk;

    while(index < config_size)
    {
        chunk = config_size - index;

        if(chunk > 32)
        {
            chunk = 32;
        }

        WriteBurst(0x5E, &config_file[index], chunk);   // INIT_DATA
        index += chunk;
    }
}

static BMI2_INTF_RETURN_TYPE bmi2_psoc_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;

    register_setting.slaveAddress = dev_addr;

    register_setting.buffer = &reg_addr;
    register_setting.bufferSize = 1;
    register_setting.xferPending = true;
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();

    register_setting.buffer = reg_data;
    register_setting.bufferSize = len;
    register_setting.xferPending = false;
    SensorBus_MasterRead(&register_setting);
    WaitForOperation();

    return BMI2_INTF_RET_SUCCESS;
}

static BMI2_INTF_RETURN_TYPE bmi2_psoc_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    uint32_t i;

    if ((len + 1) > sizeof(burstBuff))
    {
        return BMI2_E_COM_FAIL;
    }

    burstBuff[0] = reg_addr;
    for(i = 0; i < len; i++)
    {
        burstBuff[i + 1] = reg_data[i];
    }

    register_setting.slaveAddress = dev_addr;
    register_setting.buffer = burstBuff;
    register_setting.bufferSize = len + 1;
    register_setting.xferPending = false;
    SensorBus_MasterWrite(&register_setting);
    WaitForOperation();

    return BMI2_INTF_RET_SUCCESS;
}

static void bmi2_psoc_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    CyDelayUs(period);
}

char send_char[1];
cy_stc_ble_conn_handle_t appConnHandle;
bool bleConnected = false;
bool notificationsEnabled = false;

/*
*   Float Packet to be sent through BLE Notifications
*   volt_load is the voltage across a variable resistor
*   theta is the angle that the BMI270 outputs
*/
typedef struct {
    float volt_load;
    float theta;
} FloatPacket;

void bluetoothEventHandler(uint32_t event, void *eventParameter)
{
    switch(event)
    {
        case CY_BLE_EVT_STACK_ON:
        {
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            UART_PutString("BLE Stack ON, advertising...\r\n");
            break;
        }
        case CY_BLE_EVT_GAP_DEVICE_DISCONNECTED:
        {
            bleConnected = false;
            Cy_BLE_GAPP_StartAdvertisement(CY_BLE_ADVERTISING_FAST, CY_BLE_PERIPHERAL_CONFIGURATION_0_INDEX);
            UART_PutString("Disconnected, re-advertising...\r\n");
            //Buzzer Flag enabled when BLE disconnects
            buzzer_flag = 1;
            break;
        }
        case CY_BLE_EVT_GATT_CONNECT_IND:
        {
            appConnHandle = *(cy_stc_ble_conn_handle_t *)eventParameter;
            bleConnected = true;
            UART_PutString("BLE Connected!\r\n");
            //Buzzer Flag enalbed when BLE connects
            buzzer_flag = 1;
            break;
        }
        case CY_BLE_EVT_GATTS_WRITE_REQ:
        {
            cy_stc_ble_gatts_write_cmd_req_param_t *writeReqParam = 
                (cy_stc_ble_gatts_write_cmd_req_param_t *)eventParameter;

            Cy_BLE_GATTS_WriteAttributeValuePeer(&writeReqParam->connHandle, 
                                                 &writeReqParam->handleValPair);

            Cy_BLE_GATTS_WriteRsp(writeReqParam->connHandle);

            // Check if this is CCCD write
            if (writeReqParam->handleValPair.attrHandle == 
                CY_BLE_DEVICE_INTERFACE_DEVICE_OUTBOUND_CLIENT_CHARACTERISTIC_CONFIGURATION_DESC_HANDLE)
            {
                uint16_t cccdValue = writeReqParam->handleValPair.value.val[0];

                if (cccdValue & 0x0001) {
                    notificationsEnabled = true;
                    UART_PutString("Notifications ENABLED\r\n");
                } else {
                    notificationsEnabled = false;
                    UART_PutString("Notifications DISABLED\r\n");
                }
            }
        }
    }
}

void bluetoothInterruptNotify()
{
    Cy_BLE_ProcessEvents();
}

void sendBLENotification(float volt, float theta)
{
    if (!bleConnected) return;

    //initializing float packet to be sent
    FloatPacket packet;
    packet.volt_load = volt;
    packet.theta = theta;

    cy_stc_ble_gatt_handle_value_pair_t serviceHandle;
    cy_stc_ble_gatt_value_t serviceData;

    serviceData.val = (uint8_t *)&packet;
    serviceData.len = sizeof(FloatPacket);

    serviceHandle.attrHandle = CY_BLE_DEVICE_INTERFACE_DEVICE_OUTBOUND_CHAR_HANDLE;
    serviceHandle.value = serviceData;

    Cy_BLE_GATTS_WriteAttributeValueLocal(&serviceHandle);
    
    cy_en_ble_api_result_t result = Cy_BLE_GATTS_SendNotification(&appConnHandle, &serviceHandle);
    
    // Print the result so we can see what's failing
    if (result == CY_BLE_SUCCESS) {
        UART_PutString("Notification sent OK\r\n");
    } else if (result == CY_BLE_ERROR_NTF_DISABLED) {
        UART_PutString("ERROR: Notifications disabled by client\r\n");
    } else if (result == CY_BLE_ERROR_INVALID_STATE) {
        UART_PutString("ERROR: Invalid state\r\n");
    } else {
        char buf[50];
        sprintf(buf, "ERROR: code %d\r\n", result);
        UART_PutString(buf);
    }
}

int main(void)
{
    __enable_irq();

    UART_Start();
    ADC_Start();
    setvbuf(stdin, NULL, _IONBF, 0);
    
    Cy_GPIO_Write(GREEN_LED_PORT, GREEN_LED_NUM, OFF);
    Cy_GPIO_Write(YELLOW_LED_PORT, YELLOW_LED_NUM, OFF);
    Cy_GPIO_Write(RED_LED_PORT, RED_LED_NUM, OFF);

    
    SensorBus_Start();

    
    /*
    *   When BMI is not in user
    *   Comment all BMI related code, or entire program wont run
    
    bmi270_dev.intf = BMI2_I2C_INTF;
    bmi270_dev.read = bmi2_psoc_read;
    bmi270_dev.write = bmi2_psoc_write;
    bmi270_dev.delay_us = bmi2_psoc_delay_us;
    bmi270_dev.intf_ptr = &bmi270_i2c_addr;
    bmi270_dev.read_write_len = 128;
    bmi270_dev.config_file_ptr = NULL;

    printf("\r\n--- BMI270 Gesture Output ---\r\n");
    
    int8_t rslt = bmi270_init(&bmi270_dev);
    if (rslt != BMI2_OK)
    {
        printf("Init failed: %d\r\n", rslt);
        while(1) {}
    }

    uint8_t sens_list[1] = { BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 1, &bmi270_dev);
    if (rslt != BMI2_OK)
    {
        printf("Gyro enable failed: %d\r\n", rslt);
        while(1) {}
    }

    printf("Gyro ready\r\n");
    printf("Keep hand straight at startup for calibration\r\n");

    struct bmi2_sens_data sensor_data;

    float angle = 0.0f;
    float gyro_dps = 0.0f;
    float bias = 0.0f;

    char gestureChar = 'M';

    const float SENS = 0.061f;
    const float DT = 0.1f;
    const float DEAD_RATE = 1.5f;

    // -------- CALIBRATION for BMI270 --------
    for (int i = 0; i < 50; i++)
    {
        
        rslt = bmi2_get_sensor_data(&sensor_data, &bmi270_dev);
        if (rslt == BMI2_OK)
        {
            bias += sensor_data.gyr.z * SENS;
        }
        CyDelay(100);
    }

    bias /= 50.0f;
    printf("Bias: %.2f dps\r\n", bias);
    
    */
    Cy_BLE_Start(bluetoothEventHandler);

    while (Cy_BLE_GetState() != CY_BLE_STATE_ON)
    {
        Cy_BLE_ProcessEvents();
    }

    Cy_BLE_RegisterAppHostCallback(bluetoothInterruptNotify);
    
    const float VOLT_THRESHOLDS[5] = {2.50f, 2.20f, 2.45f, 1.90f, 2.60f};
    
    float volts[5]; // Create an array to hold all 5 voltage readings
    char* num_names[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"}; // Optional: To keep your exact print names
    uint8_t binary_signals[5];

    // -------- MAIN LOOP --------
    for(;;)
    {   
        if (buzzer_flag == 1) {
            Cy_GPIO_Write(BUZZER_PORT, BUZZER_NUM, ON);
            CyDelay(1000);
            Cy_GPIO_Write(BUZZER_PORT, BUZZER_NUM, OFF);
            buzzer_flag = 0;
        }
        
        /*
        rslt = bmi2_get_sensor_data(&sensor_data, &bmi270_dev);
        
        if (rslt == BMI2_OK)
        {
            gyro_dps = (sensor_data.gyr.z * SENS) - bias;

            if (gyro_dps > -DEAD_RATE && gyro_dps < DEAD_RATE)
            {
                gyro_dps = 0.0f;
            }

            angle += gyro_dps * DT;

            if (angle > 45.0f) angle = 45.0f;
            if (angle < -45.0f) angle = -45.0f;
           
            
            //sendBLENotification(gestureChar);
            //printf("CHAR: %c\r\n", gestureChar);

            // Later you can send this char to ESP32 here
            // UART_PutChar(gestureChar);
        }
        else
        {
            printf("Read failed: %d\r\n", rslt);
        }
        
        
        
        *   Variable Resistor Voltage Drop ADC Reading
        *   Potentiometer or Flex Sensor goes to ADC 0
        *   To add more Flex Sensors, utilize more ADC pins for flex Sensors
        */
        
        Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_SINGLE_SHOT);
        Cy_SAR_IsEndConversion(SAR, CY_SAR_WAIT_FOR_RESULT);
        
        uint8_t gesture_num = 0;
        
        for (uint32_t i = 0; i < 5; i++) 
        {
            // 1. Get the raw result and immediately convert it to volts
            volts[i] = Cy_SAR_CountsTo_Volts(SAR, i, Cy_SAR_GetResult16(SAR, i));
            
            
            // 2. Binary check using the corresponding threshold for this specific index 'i'
            if (volts[i] >= VOLT_THRESHOLDS[i]) {
                binary_signals[i] = 1; 
            } else {
                binary_signals[i] = 0; 
            }
            
            gesture_num += binary_signals[i]*(1 << i);
            
            // 2. Print it out
            printf("Volt_%s: %f\r\n", num_names[i], volts[i]);
        }
        
        printf("Gesture ID Hex: 0x%02X\r\n", gesture_num);
        printf("\r\n");
        
        switch (gesture_num)
        {
            case 0x00:
            case 0x10:
                //4 curled - REVERSE
                break;
            case 0x08:
            case 0x18:
                //pointing - FORWARD
                break;
            case 0x0C:
                //V - CHECK_STATUS
                break;
            case 0x11:
                //Phone - STOP
                break;
            default:
                //unrecognized or relaxed (all straight) - no command
                break;
        }
        
        //ADC for battery voltage reading
        //Quite straightforward
        /*
        int16_t batt_val = Cy_SAR_GetResult16(SAR, 1);
        float batt_val_float = Cy_SAR_CountsTo_Volts(SAR, 1, batt_val);
        printf("Battery Voltage: %f\n\r", batt_val_float);
        
        
        if (batt_val_float >= UPP_BATT) {
            Cy_GPIO_Write(GREEN_LED_PORT, GREEN_LED_NUM, ON);
            Cy_GPIO_Write(YELLOW_LED_PORT, YELLOW_LED_NUM, OFF);
            Cy_GPIO_Write(RED_LED_PORT, RED_LED_NUM, OFF);
            //UART_PutString("GREEN LED ON");
        }
        else if (batt_val_float >= MIDD_BATT && batt_val_float) {
            Cy_GPIO_Write(GREEN_LED_PORT, GREEN_LED_NUM, OFF);
            Cy_GPIO_Write(YELLOW_LED_PORT, YELLOW_LED_NUM, ON);
            Cy_GPIO_Write(RED_LED_PORT, RED_LED_NUM, OFF);
            //UART_PutString("YELLOW LED ON");
        }
        else if (batt_val_float < MIDD_BATT) {
            Cy_GPIO_Write(GREEN_LED_PORT, GREEN_LED_NUM, OFF);
            Cy_GPIO_Write(YELLOW_LED_PORT, YELLOW_LED_NUM, OFF);
            Cy_GPIO_Write(RED_LED_PORT, RED_LED_NUM, ON);
            //UART_PutString("RED LED ON");
        }
        
        */
        //Value then gets converted to a voltage float here
        //float volts = Cy_SAR_CountsTo_Volts(SAR, 0, value);
        /*
        if (volts < 0.1) {
            volts = 0.0;
        }
        else if (volts > 3.29) {
            volts = 3.3;
        }
        */
        //printf("CHAR: %c\r\n", gestureChar);
        //printf("Volts: %f\n\r", volts);
        
        //Invoke BLE Notification function
        //Pass voltage drop across var resistor and angle
        //sendBLENotification(volts,angle);
        CyDelay(100);
        //printf("CHAR: %c\r\n", gestureChar);

    }
}