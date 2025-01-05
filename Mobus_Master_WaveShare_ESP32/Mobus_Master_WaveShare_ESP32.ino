
#include "esp_log.h"
#include <SensorModbusMaster.h>
#include <HardwareSerial.h>

#define Sl_Hmi     1U
#define Sl_LvlSen  2U

#define TAG "INIT"

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define HR_SIZE_MAX 30
#define IR_SIZE_MAX 20

#define TOTAL_SLAVES 2

uint16_t HoldReg_Slxx[TOTAL_SLAVES][HR_SIZE_MAX];
uint16_t InReg_Slxx[TOTAL_SLAVES][IR_SIZE_MAX];

byte SlaveIDs[TOTAL_SLAVES] = {0x00, 0x01};

long modbusBaudRate = 19200; 
const int DEREPin = 4;

/* Define UART pins - Waveshare 6 CH Relay */
#define RS485_TXD_PIN 17 // GPIO17 for TXD
#define RS485_RXD_PIN 18 // GPIO18 for RXD

/* Use HardwareSerial for Modbus RS485 communication */
HardwareSerial* modbusSerial = &Serial2;


/* Construct the modbus instances */

modbusMaster modbusInstances[TOTAL_SLAVES];
uint16_t Slxx_ErrCount[TOTAL_SLAVES];
uint16_t Slxx_ReqCount[TOTAL_SLAVES];


void setup()
{
  pinMode(DEREPin, OUTPUT);

  modbusSerial->begin(modbusBaudRate, SERIAL_8N2, RS485_RXD_PIN, RS485_TXD_PIN);
  Serial.begin(9600);
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

  eModAdd_HR_MaxQty        = 23,
  
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
  delay(1000);
  Fetch_IR_Slave_01(Sl_Hmi);
  Fetch_IR_Slave_02(Sl_LvlSen);
  Update_RegisterValues();
}


uint8_t Fetch_IR_Slave_01(uint8_t SlaveId)
{
  /* Read Holding Registers */
  int Idx = 0;
  bool ReqStat01 = modbusInstances[SlaveId].getRegisters(eModAdd_HR_FC, eModAdd_HR_Start, eModAdd_HR_MaxQty);

  DEBUG_PRINTLN(("GetReg_HR_Sl_"+String(SlaveId)+": ")+String(ReqStat01));
  if (ReqStat01)
  {
    for (int i = 0; i < eModAdd_HR_MaxQty; i++) 
    {
      HoldReg_Slxx[SlaveId][Idx] = modbusInstances[SlaveId].uint16FromFrame(bigEndian, 3 + i * 2);
      DEBUG_PRINTLN("InReg_Sl_"+String(SlaveId)+"[" + String(i) + "]: " + String(HoldReg_Slxx[SlaveId][i]));
      Idx++;
    }
  }
  else
  {
    Slxx_ErrCount[SlaveId]++;
  }

  delay(200);

  /* Read Input Registers */

  ReqStat01 = modbusInstances[SlaveId].getRegisters(eModAdd_IR_FC, eModAdd_IR_Start, eModAdd_IR_MaxQty);
  DEBUG_PRINTLN(("GetReg_IR_Sl_"+String(SlaveId)+": ")+String(ReqStat01));
  Idx = 0;
  if (ReqStat01)
  {
    for (int i = 0; i < eModAdd_IR_MaxQty; i++) 
    {
      InReg_Slxx[SlaveId][Idx] = modbusInstances[SlaveId].uint16FromFrame(bigEndian, 3 + i * 2);
      Idx++;
    }
    for (int i = 0; i < eModAdd_IR_MaxQty; i++) 
    {
      DEBUG_PRINTLN("IR_InReg_Sl_"+String(SlaveId)+"[" + String(i) + "]: " + String(InReg_Slxx[SlaveId][i]));
    }
  }
  else
  {
    Slxx_ErrCount[SlaveId]++;
  }

  return (ReqStat01);

}

uint8_t Fetch_IR_Slave_02(uint8_t SlaveId)
{
  /* TODO: To read the value only from Input Register Address - 100, This holds the Milk Tank Level */
  return 0;
}


void Update_RegisterValues()
{
  ModValues.Get_IR_Sensor_1_CurrTemp = InReg_Slxx[Sl_Hmi][0];
  ModValues.Get_IR_Sensor_2_CurrTemp = InReg_Slxx[Sl_Hmi][2];
  ModValues.Get_IR_Curr_OnTime       = InReg_Slxx[Sl_Hmi][4];
  ModValues.Get_IR_Curr_OffTime      = InReg_Slxx[Sl_Hmi][6];
  ModValues.Get_IR_Ref_Temp          = InReg_Slxx[Sl_Hmi][8];

  ModValues.Get_HR_Set_Temp          = HoldReg_Slxx[Sl_Hmi][0];
  ModValues.Get_HR_Hys_Temp          = HoldReg_Slxx[Sl_Hmi][2];
  ModValues.Get_HR_Alarm_Hys_Temp    = HoldReg_Slxx[Sl_Hmi][4];
  ModValues.Get_HR_Stirrer_OnTime    = HoldReg_Slxx[Sl_Hmi][6];
  ModValues.Get_HR_Stirrer_OffTime   = HoldReg_Slxx[Sl_Hmi][8];
  ModValues.Get_HR_Comp_Alarm_Time   = HoldReg_Slxx[Sl_Hmi][10];
  ModValues.Get_HR_Alarm_Value       = HoldReg_Slxx[Sl_Hmi][14];
  ModValues.Get_HR_Condensor_RunSts  = HoldReg_Slxx[Sl_Hmi][21];
  ModValues.Get_HR_Stirrer_RunSts    = HoldReg_Slxx[Sl_Hmi][22];
  ModValues.Get_HR_Alarm_OnOff_Sts   = HoldReg_Slxx[Sl_Hmi][23];
  
  DEBUG_PRINTLN(("Get_IR_Sensor_1_CurrTemp = ")+String(ModValues.Get_IR_Sensor_1_CurrTemp));
  DEBUG_PRINTLN(("Get_IR_Sensor_2_CurrTemp = ")+String(ModValues.Get_IR_Sensor_2_CurrTemp));
  DEBUG_PRINTLN(("Get_IR_Curr_OnTime       = ")+String(ModValues.Get_IR_Curr_OnTime      ));
  DEBUG_PRINTLN(("Get_IR_Curr_OffTime      = ")+String(ModValues.Get_IR_Curr_OffTime     ));
  DEBUG_PRINTLN(("Get_IR_Ref_Temp          = ")+String(ModValues.Get_IR_Ref_Temp         ));
  DEBUG_PRINTLN(("Get_HR_Set_Temp          = ")+String(ModValues.Get_HR_Set_Temp         ));
  DEBUG_PRINTLN(("Get_HR_Hys_Temp          = ")+String(ModValues.Get_HR_Hys_Temp         ));
  DEBUG_PRINTLN(("Get_HR_Alarm_Hys_Temp    = ")+String(ModValues.Get_HR_Alarm_Hys_Temp   ));
  DEBUG_PRINTLN(("Get_HR_Stirrer_OnTime    = ")+String(ModValues.Get_HR_Stirrer_OnTime   ));
  DEBUG_PRINTLN(("Get_HR_Stirrer_OffTime   = ")+String(ModValues.Get_HR_Stirrer_OffTime  ));
  DEBUG_PRINTLN(("Get_HR_Comp_Alarm_Time   = ")+String(ModValues.Get_HR_Comp_Alarm_Time  ));
  DEBUG_PRINTLN(("Get_HR_Alarm_Value       = ")+String(ModValues.Get_HR_Alarm_Value      ));
  DEBUG_PRINTLN(("Get_HR_Condensor_RunSts  = ")+String(ModValues.Get_HR_Condensor_RunSts ));
  DEBUG_PRINTLN(("Get_HR_Stirrer_RunSts    = ")+String(ModValues.Get_HR_Stirrer_RunSts   ));
  DEBUG_PRINTLN(("Get_HR_Alarm_OnOff_Sts   = ")+String(ModValues.Get_HR_Alarm_OnOff_Sts  ));
};

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
