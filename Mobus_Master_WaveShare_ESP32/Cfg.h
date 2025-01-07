#include "Types.h"

/********************** * Firmware Configs *********************/

#define DEVICE_USN            "2501005"     /* Unique Serial Number for the CRRC Controller */
#define FIRMWARE_DATE         "04/12/2024"

/*********************** ModBus Configs *********************/

#define TOTAL_SLAVES     3

#define eSl_BCast        0U
#define eSl_Hmi          1U
#define eSl_LvlSen       2U

#define eSl_Id_BCast     0U
#define eSl_Id_Hmi       1U
#define eSl_Id_LvlSen    2U

byte SlaveIDs[TOTAL_SLAVES] = {eSl_Id_BCast, eSl_Id_Hmi, eSl_Id_LvlSen};

#define HR_SIZE_MAX 30
#define IR_SIZE_MAX 20

/* Define UART pins - Waveshare 6 CH Relay */
#define RS485_TXD_PIN      17          // GPIO17 for TXD
#define RS485_RXD_PIN      18          // GPIO18 for RXD
#define RS485_DERE_PIN     4
#define RS485_STOP_BITS    SERIAL_8N2  // 8 Bits, No Parity, 2 Stop Bits
#define RS485_BAUD         19200

/********************** * WiFi Configs *************************/

#define SSID_1                "Velon_2.4GHz"
#define PASSWORD_1            "Velon@070721"

#define SSID_2                "Velon_2.4GHz"
#define PASSWORD_2            "Velon@070721"

#define SSID_3                "Velonics Secure WiFi"
#define PASSWORD_3            "Velonics@2024"

#define WL_RETRY_MAX            10

/********************** * Debug Configs *************************/

#define ENABLE_DEBUG 1

#define Dbug_Serial            Serial

#if ENABLE_DEBUG
  #define DEBUG_PRINT(x) Dbug_Serial.print(x)
  #define DEBUG_PRINTLN(x) Dbug_Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

/*********************** Enable Debug Over Mqtt **************** */

#define MQTT_DIAGNOSTICS_ENABLED

/*********************** ThingSpeak Configs ******************** */

#define THINGSPEAK_DIAGNOSTICS_ENABLED

#ifdef THINGSPEAK_DIAGNOSTICS_ENABLED

#include "ThingSpeak.h"

/*
  Credentials:
  Mail ID: triboelectric.nananani@gmail.com
  Mail & ThingSpeak Password: Arun@9965658031
*/

//#define SECRET_CH_ID_CH01         2791233
//#define SECRET_WRITE_APIKEY_CH01  "ONJCE57U20EHRHPA"

#define SECRET_CH_ID_CH01         2801615
#define SECRET_WRITE_APIKEY_CH01  "ZN05N5700ZOKYSG7"

#define SECRET_CH_ID_CH02         2801615
#define SECRET_WRITE_APIKEY_CH02  "ZN05N5700ZOKYSG7"

#define SECRET_CH_ID_CH03         2801615
#define SECRET_WRITE_APIKEY_CH03  "ZN05N5700ZOKYSG7"

#define SECRET_CH_ID_CH04         2801615
#define SECRET_WRITE_APIKEY_CH04  "ZN05N5700ZOKYSG7"

#define THINGSPEAK_UPDATE_INTERVAL_MS       10000
#define THINGSPEAK_UPDATE_MAX_CH            1

#endif