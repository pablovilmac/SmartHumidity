#include "rui.h"
#include "board.h"

static RUI_RETURN_STATUS rui_return_status;
//join cnt
#define JOIN_MAX_CNT 6
static uint8_t JoinCnt=0;
RUI_LORA_STATUS_T app_lora_status; //record status 

RUI_GPIO_ST rui_gpio_2_vcc_bat; //RAK811 pin to power battery mosfet to get level
RUI_GPIO_ST rui_gpio_4_vcc_hum; //RAK811 pin to power humidity sensor
RUI_GPIO_ST rui_gpio_9_vcc_HB; //RAK811 pin to power H-Bridge
RUI_GPIO_ST rui_gpio_15_IN1; //RAK811 pin to set H-Bridge IN1
RUI_GPIO_ST rui_gpio_16_IN2; //RAK811 pin to set H-Bridge IN2
RUI_GPIO_ST rui_gpio_22_hum; //RAK811 pin to get humidity measure
RUI_GPIO_ST rui_gpio_23_bat; //RAK811 pin to get battery level (voltage divisor)

uint8_t VccBatState = 0; //Enable/disable H-Bridge 
uint8_t VccHumState = 0; //Enable/disable humidity sensor  
uint8_t VccHBState = 0; //Enable/disable H-Bridge  
uint8_t IN1 = 0; //Enable/disable H-Bridge  
uint8_t IN2 = 0; //Enable/disable H-Bridge  

int hum = 0; //Decimal humidity measure
int bat = 0; //Decimal battery level
bool watering = false;

/*******************************************************************************************
 * The BSP user functions.
 * 
 * *****************************************************************************************/ 

const uint8_t level[2]={0,1};
#define low     &level[0]
#define high    &level[1]

volatile static bool autosend_flag = false;    //auto send flag
static uint8_t a[80]={};    // Data buffer to be sent by lora
static uint8_t sensor_data_cnt=0;  //send data counter by LoRa 
bool IsJoiningflag= false;  //flag whether joining or not status
bool sample_flag = false;  //flag sensor sample record for print sensor data by AT command 
bool sample_status = false;  //current whether sample sensor completely
bool sendfull = true;  //flag whether send all sensor data 
bool firstSend = true;

void rui_lora_autosend_callback(void)  //auto_send timeout event callback
{
    autosend_flag = true;
    IsJoiningflag = false;      
}

uint8_t lpp_cnt=0;  //record lpp package count
typedef struct 
{   uint8_t startcnt;
    uint8_t size;
}lpp_data_t;
lpp_data_t lpp_data[10];

void user_lora_send(void)
{
    uint8_t dr;
    uint16_t ploadsize;
    static uint16_t temp_cnt=0;
    uint16_t temp_size=0;  //send package size 
    uint8_t* Psend_start;
    if(autosend_flag)
    {
        autosend_flag = false;
        rui_lora_get_dr(&dr,&ploadsize);
        if(ploadsize < sensor_data_cnt)
        {
            sendfull = false;  //need subcontract send
            Psend_start = &a[lpp_data[temp_cnt].startcnt];  
            if(lpp_data[temp_cnt].size > ploadsize)
            {
                RUI_LOG_PRINTF("ERROR: RUI_AT_LORA_LENGTH_ERROR %d\r\n",RUI_AT_LORA_LENGTH_ERROR);
                sample_status = false;
                sendfull = true;
                lpp_cnt = 0;
                temp_cnt = 0;
                sensor_data_cnt=0; 
                rui_lora_get_status(false,&app_lora_status); 
                switch(app_lora_status.autosend_status)
                {
                    case RUI_AUTO_ENABLE_SLEEP:rui_lora_set_send_interval(RUI_AUTO_ENABLE_SLEEP,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                        break;
                    case RUI_AUTO_ENABLE_NORMAL:rui_lora_set_send_interval(RUI_AUTO_ENABLE_NORMAL,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                        break;
                    default:break;
                }
                return; 
            }                        
            for(;temp_cnt <= lpp_cnt; temp_cnt++)
            {
                if(ploadsize < (temp_size + lpp_data[temp_cnt].size))
                {                                                              
                    rui_return_status = rui_lora_send(8,Psend_start,temp_size);
                    switch(rui_return_status)
                    {
                        case RUI_STATUS_OK:return;
                        default: RUI_LOG_PRINTF("[LoRa]: send error %d\r\n",rui_return_status);  
                            autosend_flag = true;                                      
                            break;
                    }                
                }else
                {                   
                    if(temp_cnt == lpp_cnt)
                    {
                        rui_return_status = rui_lora_send(8,Psend_start,temp_size);
                        switch(rui_return_status)
                        {
                            case RUI_STATUS_OK:RUI_LOG_PRINTF("[LoRa]: send out\r\n");
                                sample_status = false;
                                sendfull = true;
                                lpp_cnt = 0;
                                temp_cnt = 0;
                                sensor_data_cnt=0; 
                                return;
                                break;
                            default: RUI_LOG_PRINTF("[LoRa]: send error %d\r\n",rui_return_status);  
                                autosend_flag = true;                                      
                                break;
                        } 
                    }else 
                    {
                        temp_size += lpp_data[temp_cnt].size; 
                    }
                }                   
            }
        }else
        {
            rui_return_status = rui_lora_send(8,a,sensor_data_cnt);
            switch(rui_return_status)
            {
                case RUI_STATUS_OK:RUI_LOG_PRINTF("[LoRa]: send out\r\n");
                    sample_status = false;
                    sendfull = true;
                    lpp_cnt = 0;
                    sensor_data_cnt=0; 
                    break;
                default: RUI_LOG_PRINTF("[LoRa]: send error %d\r\n",rui_return_status);
                    rui_lora_get_status(false,&app_lora_status); 
                    switch(app_lora_status.autosend_status)
                    {
                        case RUI_AUTO_ENABLE_SLEEP:rui_lora_set_send_interval(RUI_AUTO_ENABLE_SLEEP,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                            break;
                        case RUI_AUTO_ENABLE_NORMAL:rui_lora_set_send_interval(RUI_AUTO_ENABLE_NORMAL,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                            break;
                        default:break;
                    }
                    break;
            }               
        }
    }                    
}

void app_loop(void)
{
    rui_lora_get_status(false,&app_lora_status);
    if(app_lora_status.IsJoined)  //if LoRaWAN is joined
    {
        if(autosend_flag)
        {
            autosend_flag=false;             
            rui_delay_ms(5);     

            //Enable Vcc humidity sensor
            VccHumState = 1;
			rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_4_vcc_hum, &VccHumState);
			rui_delay_ms(100); 

            //Get humidity value
            uint16_t value;
			rui_adc_get(&rui_gpio_22_hum, &value);

            //Calculate humidity (percent)
			uint16_t adcHum = value;
			RUI_LOG_PRINTF("\r\nHumidity adc: %d\r\n", adcHum);
            int wetValue = 1000;
            int dryValue = 3000;

            hum = 100-((100*(adcHum-wetValue))/(dryValue-wetValue));
			RUI_LOG_PRINTF("\r\nHumidity percent: %d\r\n", hum);
			
            //Disable Vcc humidity sensor
            VccHumState = 0;
			rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_4_vcc_hum, &VccHumState);

            //Get Battery Level
            /*
            uint16_t batLevelValue;
            rui_adc_get(&rui_gpio_23_bat, &batLevelValue);
            bat = (batLevelValue*0.604)-250;
            RUI_LOG_PRINTF("\r\nBattery Level: %d\r\n", bat);
            */
            
            //uint16_t voltage;
            //BoardBatteryMeasureVoltage(&voltage);
            BoardBatteryMeasureVoltage();

            if(watering){
                RUI_LOG_PRINTF("\r\nWatering: true\r\n");
            }else{
                RUI_LOG_PRINTF("\r\nWatering: false\r\n");
            }
            
            //voltage=voltage/1000.0;   //convert mV to V
            //RUI_LOG_PRINTF("Battery Voltage = %d.%d V \r\n",(uint32_t)(voltage), (uint32_t)((voltage)*1000-((int32_t)(voltage)) * 1000));
            //RUI_LOG_PRINTF("Battery Voltage = %d V \r\n",voltage);

            lpp_data[lpp_cnt].startcnt = sensor_data_cnt;

            a[sensor_data_cnt++]=0x01; //Channel 
            a[sensor_data_cnt++]=0x68; //Humidity (1 Byte)
			a[sensor_data_cnt++]=(uint16_t)hum*2; //Humidity value
            
            a[sensor_data_cnt++]=0x01; //Channel 
            a[sensor_data_cnt++]=0x66; //Digital Output (1 Byte)
			a[sensor_data_cnt++]=(uint16_t)bat; //Battery percent

            a[sensor_data_cnt++]=0x01; //Channel 
            a[sensor_data_cnt++]=0x01; //Digital Output (1 Byte)
            if(watering){
                a[sensor_data_cnt++]=0x01; //Watering flag
            }else{
                a[sensor_data_cnt++]=0x00; //Watering flag
            }
            //----------------------

            lpp_data[lpp_cnt].size = sensor_data_cnt - lpp_data[lpp_cnt].startcnt;	
            lpp_cnt++;

            if(sensor_data_cnt != 0)
            { 
                sample_status = true;                   
                sample_flag = true;
                RUI_LOG_PRINTF("\r\n");
                autosend_flag = true;
                user_lora_send();                               
            }	
            else 
            {                
                rui_lora_set_send_interval(RUI_AUTO_DISABLE,0);  //stop it auto send data if no sensor data.
            }                     
        }
    }else if(IsJoiningflag == false)
    {
        IsJoiningflag = true;
        if(app_lora_status.join_mode == RUI_OTAA)
        {
            rui_return_status = rui_lora_join();
            switch(rui_return_status)
            {
                case RUI_STATUS_OK:RUI_LOG_PRINTF("OTAA Join Start...\r\n");break;
                case RUI_LORA_STATUS_PARAMETER_INVALID:RUI_LOG_PRINTF("ERROR: RUI_AT_PARAMETER_INVALID %d\r\n",RUI_AT_PARAMETER_INVALID);
                    rui_lora_get_status(false,&app_lora_status);  //The query gets the current status 
                    switch(app_lora_status.autosend_status)
                    {
                        case RUI_AUTO_ENABLE_SLEEP:(RUI_AUTO_ENABLE_SLEEP,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                            break;
                        case RUI_AUTO_ENABLE_NORMAL:rui_lora_set_send_interval(RUI_AUTO_ENABLE_NORMAL,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                            break;
                        default:break;
                    } 
                    break;
                default: RUI_LOG_PRINTF("ERROR: LORA_STATUS_ERROR %d\r\n",rui_return_status);
                    if(app_lora_status.lora_dr > 1)rui_lora_set_dr(app_lora_status.lora_dr-1);
                    rui_lora_get_status(false,&app_lora_status); 
                    switch(app_lora_status.autosend_status)
                    {
                        case RUI_AUTO_ENABLE_SLEEP:rui_lora_set_send_interval(RUI_AUTO_ENABLE_SLEEP,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                            break;
                        case RUI_AUTO_ENABLE_NORMAL:rui_lora_set_send_interval(RUI_AUTO_ENABLE_NORMAL,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                            break;
                        default:break;
                    }
                    break;
            }            
        }
    }		
}

/*******************************************************************************************
 * LoRaMac callback functions
 * * void LoRaReceive_callback(RUI_RECEIVE_T* Receive_datapackage);//LoRaWAN callback if receive data 
 * * void LoRaP2PReceive_callback(RUI_LORAP2P_RECEIVE_T *Receive_P2Pdatapackage);//LoRaP2P callback if receive data 
 * * void LoRaWANJoined_callback(uint32_t status);//LoRaWAN callback after join server request
 * * void LoRaWANSendsucceed_callback(RUI_MCPS_T status);//LoRaWAN call back after send data complete
 * *****************************************************************************************/  
void LoRaReceive_callback(RUI_RECEIVE_T* Receive_datapackage)
{
    if(firstSend){
        RUI_LOG_PRINTF("[LoRa]: waiting for downlink msg...\r\n");
        rui_delay_ms(10000); 

        static uint8_t b[2]={};
        b[0]=0x01;  
        b[1]=0x00;
        b[2]=0x00;

        rui_return_status = rui_lora_send(8,b,3); //Send dummy message
        firstSend = false;
        switch(rui_return_status)
        {
            case RUI_STATUS_OK:
                break;
            default: RUI_LOG_PRINTF("[LoRa]: send error %d\r\n",rui_return_status);
                rui_lora_get_status(false,&app_lora_status); 
                switch(app_lora_status.autosend_status)
                {
                    case RUI_AUTO_ENABLE_SLEEP:rui_lora_set_send_interval(RUI_AUTO_ENABLE_SLEEP,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                        break;
                    case RUI_AUTO_ENABLE_NORMAL:rui_lora_set_send_interval(RUI_AUTO_ENABLE_NORMAL,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                        break;
                    default:break;
                }
                break;
        }
    }else{
        firstSend = true;
        char hex_str[3] = {0}; 
        RUI_LOG_PRINTF("at+recv=%d,%d,%d,%d", Receive_datapackage->Port, Receive_datapackage->Rssi, Receive_datapackage->Snr, Receive_datapackage->BufferSize);   
        
        if ((Receive_datapackage->Buffer != NULL) && Receive_datapackage->BufferSize) {
            RUI_LOG_PRINTF(":");
            for (int i = 0; i < Receive_datapackage->BufferSize; i++) {
                sprintf(hex_str, "%02x", Receive_datapackage->Buffer[i]);
                RUI_LOG_PRINTF("%s", hex_str); 
            }
        }
        RUI_LOG_PRINTF("\r\n");
        
        //strcmp return 0 if identical
        if(!strcmp(hex_str, "df")){
            RUI_LOG_PRINTF("[LoRa]: enable rele \r\n");

            if(!watering){
                switchRele(true);
                watering = true;
                RUI_LOG_PRINTF("[LoRa]: enabling... \r\n");
            }else{
                RUI_LOG_PRINTF("[LoRa]: already enabled \r\n");
            }

        }else if(!strcmp(hex_str, "34")){
            RUI_LOG_PRINTF("[LoRa]: disable rele \r\n");

            if(watering){
                switchRele(false);
                watering = false;
                RUI_LOG_PRINTF("[LoRa]: disabling... \r\n");
            }else{
                RUI_LOG_PRINTF("[LoRa]: already disabled \r\n");
            }
        }
    }
}

void switchRele(bool enable){
    VccHBState = 1;
    rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_9_vcc_HB, &VccHBState);
    rui_delay_ms(100); 
    if(enable == true){
        IN1 = 1;
        rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_15_IN1, &IN1);
        rui_delay_ms(50); 
        IN1 = 0;
        rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_15_IN1, &IN1);
    }else{
        IN2 = 1;
        rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_16_IN2, &IN2);
        rui_delay_ms(50); 
        IN2 = 0;
        rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_16_IN2, &IN2);
    }
    VccHBState = 0;
    rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_9_vcc_HB, &VccHBState);
}

//void BoardBatteryMeasureVoltage( uint16_t *voltage )
void BoardBatteryMeasureVoltage()
{
    VccBatState = 1;
	rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_2_vcc_bat, &VccBatState);
    rui_delay_ms(100); 
    
    uint16_t vdiv = 0;
	rui_adc_get(&rui_gpio_23_bat,&vdiv);
	
    RUI_LOG_PRINTF("\r\nBattery adc: %d\r\n", vdiv);

    double batAux = (((double)vdiv/400)-6)*((double)100/2.4);
    bat = (int)batAux;

    RUI_LOG_PRINTF("\r\nBattery percent: %d\r\n", bat);
			
    VccBatState = 0;
	rui_gpio_rw(RUI_IF_WRITE, &rui_gpio_2_vcc_bat, &VccBatState);
}

void LoRaP2PReceive_callback(RUI_LORAP2P_RECEIVE_T *Receive_P2Pdatapackage)
{
    char hex_str[3]={0};
    RUI_LOG_PRINTF("at+recv=%d,%d,%d:", Receive_P2Pdatapackage -> Rssi, Receive_P2Pdatapackage -> Snr, Receive_P2Pdatapackage -> BufferSize); 
    for(int i=0;i < Receive_P2Pdatapackage -> BufferSize; i++)
    {
        sprintf(hex_str, "%02X", Receive_P2Pdatapackage -> Buffer[i]);
        RUI_LOG_PRINTF("%s",hex_str);
    }
    RUI_LOG_PRINTF("\r\n");    
}
void LoRaWANJoined_callback(uint32_t status)
{
    static int8_t dr; 
    if(status)  //Join Success
    {
        JoinCnt = 0;
        IsJoiningflag = false;
        RUI_LOG_PRINTF("[LoRa]:Join Success\r\nOK\r\n");
        rui_lora_get_status(false,&app_lora_status);
        if(app_lora_status.autosend_status != RUI_AUTO_DISABLE)
        {
            autosend_flag = true;  //set autosend_flag after join LoRaWAN succeeded 
        }       
    }else 
    {        
        if(JoinCnt<JOIN_MAX_CNT) // Join was not successful. Try to join again
        {
            JoinCnt++;
            RUI_LOG_PRINTF("[LoRa]:Join retry Cnt:%d\r\n",JoinCnt);
            rui_lora_get_status(false,&app_lora_status);
            if(app_lora_status.lora_dr > 0)
            {
                app_lora_status.lora_dr -= 1;
            }else app_lora_status.lora_dr = 0;
            rui_lora_set_dr(app_lora_status.lora_dr);
            rui_lora_join();                    
        }
        else   //Join failed
        {
            RUI_LOG_PRINTF("ERROR: RUI_AT_LORA_INFO_STATUS_JOIN_FAIL %d\r\n",RUI_AT_LORA_INFO_STATUS_JOIN_FAIL); 
			rui_lora_get_status(false,&app_lora_status); 
            switch(app_lora_status.autosend_status)
            {
                case RUI_AUTO_ENABLE_SLEEP:rui_lora_set_send_interval(RUI_AUTO_ENABLE_SLEEP,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                    break;
                case RUI_AUTO_ENABLE_NORMAL:rui_lora_set_send_interval(RUI_AUTO_ENABLE_NORMAL,app_lora_status.lorasend_interval);  //start autosend_timer after send success
                    break;
                default:break;
            }
            JoinCnt=0;   
        }          
    }    
}
void LoRaWANSendsucceed_callback(RUI_MCPS_T mcps_type,RUI_RETURN_STATUS status)
{
    if(sendfull == false)
    {
        autosend_flag = true;
        return;
    }

    if(status == RUI_STATUS_OK)
    {
        switch( mcps_type )
        {
            case RUI_MCPS_UNCONFIRMED:
            {
                RUI_LOG_PRINTF("[LoRa]: RUI_MCPS_UNCONFIRMED send success\r\nOK\r\n");
                break;
            }
            case RUI_MCPS_CONFIRMED:
            {
                RUI_LOG_PRINTF("[LoRa]: RUI_MCPS_CONFIRMED send success\r\nOK\r\n");
                break;
            }
            case RUI_MCPS_PROPRIETARY:
            {
                RUI_LOG_PRINTF("[LoRa]: RUI_MCPS_PROPRIETARY send success\r\nOK\r\n");
                break;
            }
            case RUI_MCPS_MULTICAST:
            {
                RUI_LOG_PRINTF("[LoRa]: RUI_MCPS_MULTICAST send success\r\nOK\r\n");
                break;           
            }
            default:             
                break;
        } 
    }else RUI_LOG_PRINTF("ERROR: LORA_STATUS_ERROR %d\r\n",status);    
    
	
    rui_lora_get_status(false,&app_lora_status);//The query gets the current status 
    switch(app_lora_status.autosend_status)
    {
        case RUI_AUTO_ENABLE_SLEEP:rui_lora_set_send_interval(RUI_AUTO_ENABLE_SLEEP,app_lora_status.lorasend_interval);  //start autosend_timer after send success
            rui_delay_ms(5);  
            break;
        case RUI_AUTO_ENABLE_NORMAL:rui_lora_set_send_interval(RUI_AUTO_ENABLE_NORMAL,app_lora_status.lorasend_interval);  //start autosend_timer after send success
            break;
        default:break;
    }  
}

void LoRaP2PSendsucceed_callback(void)
{
    RUI_LOG_PRINTF("[LoRa] P2PTxDone.\r\n");    
}

/*******************************************************************************************
 * The RUI is used to receive data from uart.
 * 
 * *****************************************************************************************/ 
void rui_uart_recv(RUI_UART_DEF uart_def, uint8_t *pdata, uint16_t len)
{
    switch(uart_def)
    {
        case RUI_UART1://process code if RUI_UART1 work at RUI_UART_UNVARNISHED
            rui_lora_get_status(false,&app_lora_status);
            if(app_lora_status.IsJoined)  //if LoRaWAN is joined
            {
                rui_lora_send(8,pdata,len);
            }else
            {
                RUI_LOG_PRINTF("ERROR: RUI_AT_LORA_NO_NETWORK_JOINED %d",RUI_AT_LORA_NO_NETWORK_JOINED);
            }
             
            break;
        case RUI_UART3://process code if RUI_UART3 received data ,the len is always 1
            /*****************************************************************************
             * user code 
            ******************************************************************************/
            break;
        default:break;
    }
}

/*******************************************************************************************
 * the app_main function
 * *****************************************************************************************/ 
void main(void)
{
    rui_init();

    /*******************************************************************************************
     * Set up RAK811 battery mosfet Vcc pin
     * 
     * *****************************************************************************************/
    rui_gpio_2_vcc_bat.pin_num = 2;
    rui_gpio_2_vcc_bat.dir = RUI_GPIO_PIN_DIR_OUTPUT;
    rui_gpio_2_vcc_bat.pull = RUI_GPIO_PIN_PULLDOWN; //Ground when idle
    rui_gpio_init(&rui_gpio_2_vcc_bat);

    /*******************************************************************************************
     * Set up RAK811 humidity sensor Vcc pin
     * 
     * *****************************************************************************************/
    rui_gpio_4_vcc_hum.pin_num = 4;
    rui_gpio_4_vcc_hum.dir = RUI_GPIO_PIN_DIR_OUTPUT;
    rui_gpio_4_vcc_hum.pull = RUI_GPIO_PIN_PULLDOWN; //Ground when idle
    rui_gpio_init(&rui_gpio_4_vcc_hum);

    /*******************************************************************************************
     * Set up Switch pin
     * 
     * *****************************************************************************************/
    rui_gpio_9_vcc_HB.pin_num = 9;
    rui_gpio_9_vcc_HB.dir = RUI_GPIO_PIN_DIR_OUTPUT;
    rui_gpio_9_vcc_HB.pull = RUI_GPIO_PIN_PULLDOWN; //Ground when idle
    rui_gpio_init(&rui_gpio_9_vcc_HB);

    /*******************************************************************************************
     * Set up RAK811 H-Bridge IN1 pin
     * 
     * *****************************************************************************************/
    rui_gpio_15_IN1.pin_num = 15;
    rui_gpio_15_IN1.dir = RUI_GPIO_PIN_DIR_OUTPUT;
    rui_gpio_15_IN1.pull = RUI_GPIO_PIN_PULLDOWN; //Ground when idle
    rui_gpio_init(&rui_gpio_15_IN1);

    /*******************************************************************************************
     * Set up RAK811 H-Bridge IN2 pin
     * 
     * *****************************************************************************************/
    rui_gpio_16_IN2.pin_num = 16;
    rui_gpio_16_IN2.dir = RUI_GPIO_PIN_DIR_OUTPUT;
    rui_gpio_16_IN2.pull = RUI_GPIO_PIN_PULLDOWN; //Ground when idle
    rui_gpio_init(&rui_gpio_16_IN2);

    /*******************************************************************************************
     * Set up RAK811 humidity sensor measure pin
     * 
     * *****************************************************************************************/
    rui_gpio_22_hum.pin_num = 22;
    rui_gpio_22_hum.dir = RUI_GPIO_PIN_DIR_INPUT;
    rui_gpio_22_hum.pull = RUI_GPIO_PIN_NOPULL;
    rui_adc_init(&rui_gpio_22_hum);

    /*******************************************************************************************
     * Set up RAK811 battery level pin
     * 
     * *****************************************************************************************/
    rui_gpio_23_bat.pin_num = 23;
    rui_gpio_23_bat.dir = RUI_GPIO_PIN_DIR_INPUT;
    rui_gpio_23_bat.pull = RUI_GPIO_PIN_PULLDOWN;
    rui_adc_init(&rui_gpio_23_bat);

    /*******************************************************************************************
     * Register LoRaMac callback function
     * 
     * *****************************************************************************************/
    rui_lora_register_recv_callback(LoRaReceive_callback);  
    rui_lorap2p_register_recv_callback(LoRaP2PReceive_callback);
    rui_lorajoin_register_callback(LoRaWANJoined_callback); 
    rui_lorasend_complete_register_callback(LoRaWANSendsucceed_callback); 
    rui_lorap2p_complete_register_callback(LoRaP2PSendsucceed_callback);

    /*******************************************************************************************    
     *The query gets the current status 
    * 
    * *****************************************************************************************/ 
    rui_lora_get_status(false,&app_lora_status);

	if(app_lora_status.autosend_status)RUI_LOG_PRINTF("autosend_interval: %us\r\n", app_lora_status.lorasend_interval);

    /*******************************************************************************************    
     *Init OK ,print board status and auto join LoRaWAN
    * 
    * *****************************************************************************************/  
    switch(app_lora_status.work_mode)
	{
		case RUI_LORAWAN:
            RUI_LOG_PRINTF("Initialization OK,Current work_mode:LoRaWAN,");
            if(app_lora_status.join_mode == RUI_OTAA)
            {
                RUI_LOG_PRINTF(" join_mode:OTAA,");  
                switch(app_lora_status.class_status)
                {
                    case RUI_CLASS_A:RUI_LOG_PRINTF(" Class: A\r\n");
                        break;
                    case RUI_CLASS_B:RUI_LOG_PRINTF(" Class: B\r\n");
                        break;
                    case RUI_CLASS_C:RUI_LOG_PRINTF(" Class: C\r\n");
                        break;
                    default:break;
                }              
            }else if(app_lora_status.join_mode == RUI_ABP)
            {
                RUI_LOG_PRINTF(" join_mode:ABP,\r\n");
                switch(app_lora_status.class_status)
                {
                    case RUI_CLASS_A:RUI_LOG_PRINTF(" Class: A\r\n");
                        break;
                    case RUI_CLASS_B:RUI_LOG_PRINTF(" Class: B\r\n");
                        break;
                    case RUI_CLASS_C:RUI_LOG_PRINTF(" Class: C\r\n");
                        break;
                    default:break;
                } 
                if(rui_lora_join() == RUI_STATUS_OK)//join LoRaWAN by ABP mode
                {
                    LoRaWANJoined_callback(1);  //ABP mode join success
                }
            }
			break;
		case RUI_P2P:RUI_LOG_PRINTF("Current work_mode:P2P\r\n");
			break;
		default: break;
	}   
    RUI_LOG_PRINTF("\r\n");

    switchRele(false);

    while(1)
    {       
        rui_lora_get_status(false,&app_lora_status);//The query gets the current status 
        rui_running();
        switch(app_lora_status.work_mode)
        {
            case RUI_LORAWAN:
                if(!sample_status)app_loop();
                else user_lora_send();			
                break; 
            case RUI_P2P:
                /*************************************************************************************
                 * user code at LoRa P2P mode
                *************************************************************************************/
                break;
            default :break;
        }
    }
}
