
#include <Cfg.h>

#include "esp_log.h"
#include <SensorModbusMaster.h>
#include <HardwareSerial.h>

#define TAG "INIT"

/************************* Globals - End **********************************/

#include <WiFi.h>
#include <WiFiMulti.h>

WiFiMulti WiFiMulti;

uint16_t Wifi_TryCount = 0;

/* Use HardwareSerial for Modbus RS485 communication */
HardwareSerial* modbusSerial = &Serial2;

/* Construct the modbus instances */
modbusMaster modbusInstances[TOTAL_SLAVES];

uint16_t HoldReg_Slxx[TOTAL_SLAVES][HR_SIZE_MAX];
uint16_t InReg_Slxx[TOTAL_SLAVES][IR_SIZE_MAX];

uint8_t Slxx_ErrCount[TOTAL_SLAVES];
uint8_t Slxx_ReqCount[TOTAL_SLAVES];

uint8_t Slxx_ErrState[TOTAL_SLAVES];

uint32_t TS_GRP_01 = 0;
uint32_t TS_GRP_02 = 0;
uint32_t TS_GRP_03 = 0;
uint32_t TS_GRP_04 = 0;


/* ****************** ThingSpeak Diagnostics - Begin ********************************/

#ifdef THINGSPEAK_DIAGNOSTICS_ENABLED

unsigned long myChannelNumber_CH01 = SECRET_CH_ID_CH01;
const char * myWriteAPIKey_CH01 = SECRET_WRITE_APIKEY_CH01;

unsigned long myChannelNumber_CH02 = SECRET_CH_ID_CH02;
const char * myWriteAPIKey_CH02 = SECRET_WRITE_APIKEY_CH02;

unsigned long myChannelNumber_CH03 = SECRET_CH_ID_CH03;
const char * myWriteAPIKey_CH03 = SECRET_WRITE_APIKEY_CH03;

unsigned long myChannelNumber_CH04 = SECRET_CH_ID_CH04;
const char * myWriteAPIKey_CH04 = SECRET_WRITE_APIKEY_CH04;

WiFiClient TS_client;

/* ThingSpeak Init - Begin */
void ThingSpeak_Init(void)
{
  ThingSpeak.begin(TS_client);
}
/* ThingSpeak Init - End */


/* ThingSpeak Update - Begin */

unsigned long ThingSpeak_lastSendTime = 0;

float CH01_Values[8] = {0};
float CH02_Values[8] = {0};
float CH03_Values[8] = {0};
float CH04_Values[8] = {0};

String CH01_Status = "NA";
String CH02_Status = "NA";
String CH03_Status = "NA";
String CH04_Status = "NA";

Std_RetType ThingSpeak_Update_CH01()
{
  return (ThingSpeak_Update_CHXX(myChannelNumber_CH01, myWriteAPIKey_CH01, CH01_Values, CH01_Status));
}

Std_RetType ThingSpeak_Update_CH02()
{
  return (ThingSpeak_Update_CHXX(myChannelNumber_CH02, myWriteAPIKey_CH02, CH02_Values, CH02_Status));
}

Std_RetType ThingSpeak_Update_CH03()
{
  return (ThingSpeak_Update_CHXX(myChannelNumber_CH03, myWriteAPIKey_CH03, CH03_Values, CH03_Status));
}

Std_RetType ThingSpeak_Update_CH04()
{
  return (ThingSpeak_Update_CHXX(myChannelNumber_CH04, myWriteAPIKey_CH04, CH04_Values, CH04_Status));
}

Std_RetType ThingSpeak_Update_CHXX(int CHXX_No, const char * CHXX_WriteAPIKey, float * CHXX_Values, String CHXX_Status)
{

  Std_RetType Ret = E_NOT_OK;

  for (int i = 0; i < 8; i++)
  {
    ThingSpeak.setField(i + 1, CHXX_Values[i]);
  }

  ThingSpeak.setStatus(CHXX_Status);

  int x = ThingSpeak.writeFields(CHXX_No, CHXX_WriteAPIKey);

  if (x == 200)
  {
    DEBUG_PRINTLN("[TS]: CH-" + String(CHXX_No) + " Update Success !");
    Ret = E_OK;
  }
  else
  {
    DEBUG_PRINTLN("[TS]: CH-" + String(CHXX_No) + " Update Failed ! - EC: " + String(x));
  }

  return Ret;
}

#endif

/* ****************** ThingSpeak Diagnostics - End ********************************/

void setup()
{
  pinMode(RS485_DERE_PIN, OUTPUT);
  modbusSerial->begin(RS485_BAUD, RS485_STOP_BITS, RS485_RXD_PIN, RS485_TXD_PIN);

  #if ENABLE_DEBUG
  Dbug_Serial.begin(9600);
  #endif

  WiFi_Init();

  /* Start the modbus instances */
  for (int i = 0; i < TOTAL_SLAVES; ++i)
  {
    modbusInstances[i].begin(SlaveIDs[i], modbusSerial, RS485_DERE_PIN);
  }

  DEBUG_PRINT("Initializing the system!");

  #ifdef THINGSPEAK_DIAGNOSTICS_ENABLED
  ThingSpeak_Init();
  #endif

}

ModValues_Struct ModValues;

void loop()
{
  WiFi_Loop();
  ModReq_Sl_Hmi();
  ModReq_Sl_LvlSen();
  Update_RegisterValues();
  delay(1000);

  #ifdef THINGSPEAK_DIAGNOSTICS_ENABLED
  loop_ThingSpeak();
  #endif
}


uint8_t ModReq_Sl_Hmi()
{
  uint8_t Slave = eSl_Hmi;
  /* Read Holding Registers */

  Slxx_ErrState[Slave] = eNoError;

  int Idx = 0;
  bool ReqStat01 = modbusInstances[Slave].getRegisters(eModAdd_HR_FC, eModAdd_HR_Start, eModAdd_HR_MaxQty);

  DEBUG_PRINTLN(("GetReg_HR_Sl_"+String(Slave)+": ")+String(ReqStat01));
  if (ReqStat01)
  {
    for (int i = 0; i < eModAdd_HR_MaxQty; i++) 
    {
      HoldReg_Slxx[Slave][Idx] = modbusInstances[Slave].uint16FromFrame(bigEndian, 3 + i * 2);
      DEBUG_PRINTLN("InReg_Sl_"+String(Slave)+"[" + String(i) + "]: " + String(HoldReg_Slxx[Slave][i]));
      Idx++;
    }
  }
  else
  {
    Slxx_ErrCount[Slave]++;
    Slxx_ErrState[Slave] = eError;
  }

  delay(200);

  /* Read Input Registers */

  bool ReqStat02 = modbusInstances[Slave].getRegisters(eModAdd_IR_FC, eModAdd_IR_Start, eModAdd_IR_MaxQty);
  DEBUG_PRINTLN(("GetReg_IR_Sl_"+String(Slave)+": ")+String(ReqStat02));
  Idx = 0;
  if (ReqStat02)
  {
    for (int i = 0; i < eModAdd_IR_MaxQty; i++) 
    {
      InReg_Slxx[Slave][Idx] = modbusInstances[Slave].uint16FromFrame(bigEndian, 3 + i * 2);
      Idx++;
    }
    for (int i = 0; i < eModAdd_IR_MaxQty; i++) 
    {
      DEBUG_PRINTLN("IR_InReg_Sl_"+String(Slave)+"[" + String(i) + "]: " + String(InReg_Slxx[Slave][i]));
    }
  }
  else
  {
    Slxx_ErrCount[Slave]++;
    Slxx_ErrState[Slave] = eError;
  }

  return (ReqStat01 & ReqStat02);

}

uint8_t ModReq_Sl_LvlSen()
{
  uint8_t Slave = eSl_LvlSen;

  /* Read Input Registers */
  
  Slxx_ErrState[Slave] = eNoError;

  bool ReqStat01 = modbusInstances[Slave].getRegisters(eModAdd_IR_FC, eModAdd_LvlSen_IR_Start, eModAdd_LvlSen_IR_MaxQty);
  DEBUG_PRINTLN(("GetReg_IR_Sl_"+String(Slave)+": ")+String(ReqStat01));
  int Idx = 0;
  if (ReqStat01)
  {
    for (int i = 0; i < eModAdd_LvlSen_IR_MaxQty; i++) 
    {
      InReg_Slxx[Slave][Idx] = modbusInstances[Slave].uint16FromFrame(bigEndian, 3 + i * 2);
      Idx++;
    }
    for (int i = 0; i < eModAdd_LvlSen_IR_MaxQty; i++) 
    {
      DEBUG_PRINTLN("IR_InReg_Sl_"+String(Slave)+"[" + String(i) + "]: " + String(InReg_Slxx[Slave][i]));
    }
  }
  else
  {
    Slxx_ErrCount[Slave]++;
    Slxx_ErrState[Slave] = eError;
  }

  return (ReqStat01);
}


void Update_RegisterValues()
{

  ModValues.Get_HR_Set_Temp          = HoldReg_Slxx[eSl_Hmi][0];
  ModValues.Get_HR_Hys_Temp          = HoldReg_Slxx[eSl_Hmi][2];
  ModValues.Get_HR_Alarm_Hys_Temp    = HoldReg_Slxx[eSl_Hmi][4];
  ModValues.Get_HR_Stirrer_OnTime    = HoldReg_Slxx[eSl_Hmi][6];
  ModValues.Get_HR_Stirrer_OffTime   = HoldReg_Slxx[eSl_Hmi][8];
  ModValues.Get_HR_Comp_Alarm_Time   = HoldReg_Slxx[eSl_Hmi][10];
  ModValues.Get_HR_Alarm_Value       = HoldReg_Slxx[eSl_Hmi][14];
  ModValues.Get_HR_Condensor_RunSts  = HoldReg_Slxx[eSl_Hmi][21];
  ModValues.Get_HR_Stirrer_RunSts    = HoldReg_Slxx[eSl_Hmi][22];
  ModValues.Get_HR_Alarm_OnOff_Sts   = HoldReg_Slxx[eSl_Hmi][23];

  ModValues.Get_IR_Sensor_1_CurrTemp = InReg_Slxx[eSl_Hmi][0];
  ModValues.Get_IR_Sensor_2_CurrTemp = InReg_Slxx[eSl_Hmi][2];
  ModValues.Get_IR_Curr_OnTime       = InReg_Slxx[eSl_Hmi][4];
  ModValues.Get_IR_Curr_OffTime      = InReg_Slxx[eSl_Hmi][6];
  ModValues.Get_IR_Ref_Temp          = InReg_Slxx[eSl_Hmi][8];

  DEBUG_PRINTLN(("Slxx_ErrCount[eSl_Hmi]: ") + String(Slxx_ErrCount[eSl_Hmi]));

  /* GRP_01 - Begin */

  DEBUG_PRINTLN(("Get_HR_Set_Temp          = ")+String(ModValues.Get_HR_Set_Temp         ));
  DEBUG_PRINTLN(("Get_HR_Hys_Temp          = ")+String(ModValues.Get_HR_Hys_Temp         ));
  DEBUG_PRINTLN(("Get_HR_Alarm_Hys_Temp    = ")+String(ModValues.Get_HR_Alarm_Hys_Temp   ));
  DEBUG_PRINTLN(("Get_HR_Stirrer_OnTime    = ")+String(ModValues.Get_HR_Stirrer_OnTime   ));

  TS_GRP_01 =  ((ModValues.Get_HR_Set_Temp         & 0xFF) << 0 ) |
               ((ModValues.Get_HR_Hys_Temp         & 0xFF) << 8 ) |
               ((ModValues.Get_HR_Alarm_Hys_Temp   & 0xFF) << 16) |
               ((ModValues.Get_HR_Stirrer_OnTime   & 0xFF) << 24);
  /* GRP_01 - End */

  /* GRP_02 - Begin */

  DEBUG_PRINTLN(("Get_HR_Stirrer_OffTime   = ")+String(ModValues.Get_HR_Stirrer_OffTime  ));
  DEBUG_PRINTLN(("Get_HR_Comp_Alarm_Time   = ")+String(ModValues.Get_HR_Comp_Alarm_Time  ));
  DEBUG_PRINTLN(("Get_IR_Ref_Temp          = ")+String(ModValues.Get_IR_Ref_Temp         ));
  DEBUG_PRINTLN(("Get_HR_Alarm_Value       = ")+String(ModValues.Get_HR_Alarm_Value      ));
  DEBUG_PRINTLN(("Get_HR_Condensor_RunSts  = ")+String(ModValues.Get_HR_Condensor_RunSts ));
  DEBUG_PRINTLN(("Get_HR_Stirrer_RunSts    = ")+String(ModValues.Get_HR_Stirrer_RunSts   ));
  DEBUG_PRINTLN(("Get_HR_Alarm_OnOff_Sts   = ")+String(ModValues.Get_HR_Alarm_OnOff_Sts  ));

  TS_GRP_02 =  ((ModValues.Get_HR_Stirrer_OffTime  & 0xFF) << 0 )  |  // LSB (8 bits)
               ((ModValues.Get_HR_Comp_Alarm_Time  & 0xFF) << 8 )  |  // Next 8 bits
               ((ModValues.Get_IR_Ref_Temp         & 0xFF) << 16)  |  // Next 8 bits
               ((ModValues.Get_HR_Alarm_Value      & 0x0F) << 24)  |  // Next 4 bits
               ((ModValues.Get_HR_Condensor_RunSts & 0x01) << 28)  |  // 1 bit at position 28
               ((ModValues.Get_HR_Stirrer_RunSts   & 0x01) << 29)  |  // 1 bit at position 29
               ((ModValues.Get_HR_Alarm_OnOff_Sts  & 0x01) << 30);    // 1 bit at position 30
  /* GRP_02 - End */


  /* GRP_03 - Begin */

  DEBUG_PRINTLN(("Get_IR_Sensor_1_CurrTemp = ")+String(ModValues.Get_IR_Sensor_1_CurrTemp));
  DEBUG_PRINTLN(("Get_IR_Sensor_2_CurrTemp = ")+String(ModValues.Get_IR_Sensor_2_CurrTemp));
  DEBUG_PRINTLN(("Get_IR_Curr_OnTime       = ")+String(ModValues.Get_IR_Curr_OnTime      ));
  DEBUG_PRINTLN(("Get_IR_Curr_OffTime      = ")+String(ModValues.Get_IR_Curr_OffTime     ));

  uint32_t TS_GRP_03 =  ((ModValues.Get_IR_Sensor_1_CurrTemp  & 0xFF) << 0 ) |
                        ((ModValues.Get_IR_Sensor_2_CurrTemp  & 0xFF) << 8 ) |
                        ((ModValues.Get_IR_Curr_OnTime        & 0xFF) << 16) |
                        ((ModValues.Get_IR_Curr_OffTime       & 0xFF) << 24);
  /* GRP_03 - End */

  /* GRP_04 - Begin */

  DEBUG_PRINTLN(("Slxx_ErrCount[eSl_Hmi]    = ")+String(Slxx_ErrCount[eSl_Hmi]));
  DEBUG_PRINTLN(("Slxx_ErrCount[eSl_LvlSen] = ")+String(Slxx_ErrCount[eSl_LvlSen]));

  DEBUG_PRINTLN(("Slxx_ErrState[eSl_Hmi]    = ")+String(Slxx_ErrState[eSl_Hmi]));
  DEBUG_PRINTLN(("Slxx_ErrState[eSl_LvlSen] = ")+String(Slxx_ErrState[eSl_LvlSen]));

  TS_GRP_04 =  ((Slxx_ErrCount[eSl_Hmi]      & 0xFF) << 0 ) |
               ((Slxx_ErrCount[eSl_LvlSen]   & 0xFF) << 8 ) |
               ((Slxx_ErrState[eSl_Hmi]      & 0x01) << 16) |
               ((Slxx_ErrState[eSl_LvlSen]   & 0x01) << 17);
  /* GRP_04 - End */

  CH01_Values[3] = TS_GRP_01; /* Field 4 */
  CH01_Values[4] = TS_GRP_02; /* Field 5 */
  CH01_Values[5] = TS_GRP_03; /* Field 6 */
  CH01_Values[6] = TS_GRP_04; /* Field 7 */
  
};

/* ****************** Reset ********************************/

void Perform_McuReset()
{
  esp_restart();
}


/* ****************** WiFi ********************************/

void WiFi_Init()
{
  WiFiMulti.addAP(SSID_1, PASSWORD_1);
  WiFiMulti.addAP(SSID_2, PASSWORD_2);
  WiFiMulti.addAP(SSID_3, PASSWORD_3);

  DEBUG_PRINTLN("Connecting to WiFi...");
  WiFi_Loop();
}

void WiFi_Loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_PRINT("Attempting to connect to WiFi...");

    Wifi_TryCount = 0;

    delay(2000);

    while (WiFiMulti.run() != WL_CONNECTED && Wifi_TryCount < WL_RETRY_MAX)
    {
      DEBUG_PRINT(".");

      delay(5000);

      Wifi_TryCount++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      DEBUG_PRINT("Connected to SSID: " + String(WiFi.SSID()));
      DEBUG_PRINTLN("RSSI: " + String(WiFi.RSSI()));
      DEBUG_PRINTLN("IP address: " + String(WiFi.localIP()));

      Wifi_TryCount = 0;
    }
    else
    {
      DEBUG_PRINTLN("\nFailed to Connect, Resetting MCU...");
      Perform_McuReset();
    }
  }
}


#ifdef THINGSPEAK_DIAGNOSTICS_ENABLED

void loop_ThingSpeak()
{
  Std_RetType Ret = E_NOT_OK;

  if (millis() - ThingSpeak_lastSendTime >= THINGSPEAK_UPDATE_INTERVAL_MS)
  {
    Ret = ThingSpeak_Update_CH01();
    ThingSpeak_lastSendTime = millis();
  }
  /* ThingSpeak_lastSendTime will be updated in the Func ThingSpeak_Update */
}

float GetUptime()
{
  unsigned long totalSeconds = millis() / 1000;
  unsigned long minutes = totalSeconds / 60;
  unsigned long seconds = totalSeconds % 60;

  float decimalSeconds = seconds / 100.0;
  return minutes + decimalSeconds;
}

#endif


