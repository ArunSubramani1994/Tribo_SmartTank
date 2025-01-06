#include <Arduino.h>

#define TRUE        1
#define FALSE       0

#define eNoError    0U
#define eError      1U

typedef uint8_t Std_RetType;

#define E_OK        0
#define E_NOT_OK    1

#define INDEX_0     0
#define INDEX_1     1
#define INDEX_2     2
#define INDEX_3     3
#define INDEX_4     4
#define INDEX_5     5
#define INDEX_6     6
#define INDEX_7     7


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

#define THINGSPEAK_DIAGNOSTICS_ENABLED

#ifdef THINGSPEAK_DIAGNOSTICS_ENABLED

#define CH00    0
#define CH01    1
#define CH02    2
#define CH03    3
#define CH04    4

#endif