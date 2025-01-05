
#include "esp_log.h"
#include <SensorModbusMaster.h>
#include <HardwareSerial.h>

#define eNoError    0U
#define eError      1U

#define TAG "INIT"

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define TOTAL_SLAVES 3

#define eSl_BCast        0U
#define eSl_Hmi          1U
#define eSl_LvlSen       2U

#define eSl_Id_BCast     0U
#define eSl_Id_Hmi       1U
#define eSl_Id_LvlSen    2U

byte SlaveIDs[TOTAL_SLAVES] = {eSl_Id_BCast, eSl_Id_Hmi, eSl_Id_LvlSen};

#define HR_SIZE_MAX 30
#define IR_SIZE_MAX 20

long modbusBaudRate = 19200; 
const int DEREPin = 4;

/* Define UART pins - Waveshare 6 CH Relay */
#define RS485_TXD_PIN 17 // GPIO17 for TXD
#define RS485_RXD_PIN 18 // GPIO18 for RXD

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

void setup()
{
  pinMode(DEREPin, OUTPUT);

  modbusSerial->begin(modbusBaudRate, SERIAL_8N2, RS485_RXD_PIN, RS485_TXD_PIN);

  #if ENABLE_DEBUG
  Serial.begin(9600);
  #endif

  /* Start the modbus instances */
  for (int i = 0; i < TOTAL_SLAVES; ++i)
  {
    modbusInstances[i].begin(SlaveIDs[i], modbusSerial, DEREPin);
  }

  DEBUG_PRINT("Initializing the system!");

}

typedef enum
{
  eModAdd_Coil_FC                 = 0x01,
  eModAdd_IR_FC                   = 0x04,
  eModAdd_HR_FC                   = 0x03,

  eModAdd_Coil_Start              = 0,
  eModAdd_Coil_Condensor          = 0,
  eModAdd_Coil_Stirrer            = 1,
  eModAdd_Coil_Alarm_Off          = 2,
  eModAdd_Coil_Alarm_Reset        = 3,
  eModAdd_Coil_MaxQty             = 4,

  eModAdd_IR_Start                = 100,
  eModAdd_IR_Sensor_1_CurrTemp    = 100,
  eModAdd_IR_Sensor_2_CurrTemp    = 102,
  eModAdd_IR_Curr_OnTime          = 104,
  eModAdd_IR_Curr_OffTime         = 106,
  eModAdd_IR_Ref_Temp             = 108,
  eModAdd_IR_MaxQty               = 10,

  eModAdd_HR_Start                = 210,
  eModAdd_HR_Set_Temp             = 210,
  eModAdd_HR_Hys_Temp             = 212,
  eModAdd_HR_Alarm_Hys_Temp       = 214,
  eModAdd_HR_Stirrer_OnTime       = 216,
  eModAdd_HR_Stirrer_OffTime      = 218,
  eModAdd_HR_Comp_Alarm_Time      = 220,

  eModAdd_HR_Alarm_Value          = 224,
  eModAdd_HR_Condensor_RunSts     = 230,
  eModAdd_HR_Stirrer_RunSts       = 231,
  eModAdd_HR_Alarm_OnOff_Sts      = 232,

  eModAdd_HR_MaxQty               = 23,

  eModAdd_LvlSen_IR_Start         = 100,
  eModAdd_LvlSen_IR_TankLvl       = 100,
  eModAdd_LvlSen_IR_MaxQty        = 0,

} EModAdd;

typedef struct ModValues_Struct
{
  uint32_t Get_IR_Sensor_1_CurrTemp;
  uint32_t Get_IR_Sensor_2_CurrTemp;
  uint32_t Get_IR_Curr_OnTime;
  uint32_t Get_IR_Curr_OffTime;
  uint32_t Get_IR_Ref_Temp;

  uint32_t Get_HR_Set_Temp;
  uint32_t Get_HR_Hys_Temp;
  uint32_t Get_HR_Alarm_Hys_Temp;
  uint32_t Get_HR_Stirrer_OnTime;
  uint32_t Get_HR_Stirrer_OffTime;
  uint32_t Get_HR_Comp_Alarm_Time;

  uint16_t Get_HR_Alarm_Value;
  uint16_t Get_HR_Condensor_RunSts;
  uint16_t Get_HR_Stirrer_RunSts;
  uint16_t Get_HR_Alarm_OnOff_Sts;
};

ModValues_Struct ModValues;


void loop()
{
  ModReq_Sl_Hmi();
  ModReq_Sl_LvlSen();
  Update_RegisterValues();
  delay(1000);
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

};

float GetUptime()
{
  unsigned long totalSeconds = millis() / 1000;
  unsigned long minutes = totalSeconds / 60;
  unsigned long seconds = totalSeconds % 60;

  float decimalSeconds = seconds / 100.0;
  return minutes + decimalSeconds;
}

/*
Field 1 - UpTime
Field 2 - Tank Level Avg
Field 3 - Tank Level Raw
Field 4 - GRP_01
Field 5 - GRP_02
Field 6 - GRP_03
Field 7 - GRP_04
Field 8 - GRP_05
*/
