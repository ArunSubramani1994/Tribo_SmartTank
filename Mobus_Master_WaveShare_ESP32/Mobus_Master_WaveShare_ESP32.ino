
#include "esp_log.h"
#include <SensorModbusMaster.h>
#include <HardwareSerial.h>

#define TAG "INIT"

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

#define HR_SIZE_MAX 20
#define IR_SIZE_MAX 20

#define TOTAL_SLAVES 2

uint32_t HoldReg_Slxx[TOTAL_SLAVES][HR_SIZE_MAX];
uint32_t InReg_Slxx[TOTAL_SLAVES][IR_SIZE_MAX];

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
  eModAdd_HR_Uint32_MaxQty        = 12,

  eModAdd_HR_Alarm_Value          = 224,
  eModAdd_HR_MaxQty               = 15
  
} EModAdd;


void loop()
{
  delay(1000);
  Fetch_IR_Slave_01(1);
  Fetch_IR_Slave_02(2);
}


uint8_t Fetch_IR_Slave_01(uint8_t SlaveId)
{

  bool ReqStat01 = modbusInstances[SlaveId].getRegisters(eModAdd_IR_FC, eModAdd_IR_Start, eModAdd_IR_MaxQty);

  DEBUG_PRINTLN(("GetReg_IR_Sl_"+String(SlaveId)+": ")+String(ReqStat01));
  int Idx = 0;
  if (ReqStat01)
  {
    for (int i = 0; i < eModAdd_IR_MaxQty; i++) 
    {
      InReg_Slxx[SlaveId][Idx] = modbusInstances[SlaveId].uint32FromFrame(littleEndian, 3 + i * 2);
      i++;
      Idx++;
    }
    for (int i = 0; i < eModAdd_IR_MaxQty/2; i++) 
    {
      DEBUG_PRINTLN("InReg_Sl_"+String(SlaveId)+"[" + String(i) + "]: " + String(InReg_Slxx[SlaveId][i]));
    }
  }
  else
  {
    Slxx_ErrCount[SlaveId]++;
  }

  Idx = 0;
  ReqStat01 = modbusInstances[SlaveId].getRegisters(eModAdd_HR_FC, eModAdd_HR_Start, eModAdd_HR_MaxQty);

  DEBUG_PRINTLN(("GetReg_HR_Sl_"+String(SlaveId)+": ")+String(ReqStat01));
  if (ReqStat01)
  {
    for (int i = 0; i < eModAdd_HR_Uint32_MaxQty; i++) 
    {
      HoldReg_Slxx[SlaveId][Idx] = modbusInstances[SlaveId].uint32FromFrame(littleEndian, 3 + i * 2);
      i++;
      Idx++;
    }

    /* eModAdd_HR_Alarm_Value is uint16 Value */
    HoldReg_Slxx[SlaveId][6] = modbusInstances[SlaveId].uint16FromFrame(littleEndian, 3 + 14 * 2);

    for (int i = 0; i < (eModAdd_HR_Uint32_MaxQty/2)+1; i++) 
    {
      DEBUG_PRINTLN("InReg_Sl_"+String(SlaveId)+"[" + String(i) + "]: " + String(HoldReg_Slxx[SlaveId][i]));
    }
  }
  else
  {
    Slxx_ErrCount[SlaveId]++;
  }

  /* TODO: Coil Status also need to be Implemented */

  
  return (ReqStat01);
  
}

uint8_t Fetch_IR_Slave_02(uint8_t SlaveId)
{
  /* TODO: To read the value only from Input Register Address - 100, This holds the Milk Tank Level */
  return 0;
}

