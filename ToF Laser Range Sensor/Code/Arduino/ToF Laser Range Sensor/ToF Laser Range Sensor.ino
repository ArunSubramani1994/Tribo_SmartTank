#ifdef BLUETOOTH_DIAGNOSTICS_ENABLED
  #include "BluetoothSerial.h"
#endif

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#define RXD2           16   // UART2 RX pin (connect to sensor's TX)
#define TXD2           17   // UART2 TX pin (connect to sensor's RX)
#define BUFFER_SIZE    100  // Number of values to average
#define MAX_DIFFERENCE 5.0  // Maximum allowed difference in cm

#define MQTT_DIAGNOSTICS_ENABLED
#define THINKSPEAK_DIAGNOSTICS_ENABLED

#define WL_RETRY_MAX  5

uint16_t Wifi_TryCount = 0;

const char *ssid = "Velonics Secure WiFi";
const char *pass = "Velonics@2024";

float avgDistance = 0;

/*********************** MQTT Diagnostics - Begin *********************/

#ifdef THINKSPEAK_DIAGNOSTICS_ENABLED

/* ThingSpeak - Begin */

#include "ThingSpeak.h"


/*
  Credentials:
  Mail ID: triboelectric.nananani@gmail.com
  Mail & ThingSpeak Password: Arun@9965658031
*/


/* TriboElectric - Nana Nani */

#define SECRET_CH_ID_CH01          2791233
#define SECRET_WRITE_APIKEY_CH01  "ONJCE57U20EHRHPA" 

unsigned long myChannelNumber_CH01 = SECRET_CH_ID_CH01;
const char * myWriteAPIKey_CH01 = SECRET_WRITE_APIKEY_CH01;


WiFiClient TS_client;

/* ThingSpeak - End */

#endif

/*********************** MQTT Diagnostics - Begin *********************/

#ifdef MQTT_DIAGNOSTICS_ENABLED

/*
  Credentials:
  Mail ID: triboelectric.crrc@gmail.com
  Mail & ThingSpeak Password: Arun@9965658031
*/

#include <PubSubClient.h>

const unsigned int mqtt_port = 8883;  // MQTT port (TLS)
const char *mqtt_broker = "d3718ccc.ala.us-east-1.emqxsl.com";  // EMQX broker endpoint
const char *mqtt_topic_Pub = "TE/NaNa/Pub/";     // MQTT topic
const char *mqtt_topic_Sub = "TE/NaNa/Sub/#";   // MQTT topic
const char *mqtt_username = "Admin";  // MQTT username for authentication
const char *mqtt_password = "Admin";  // MQTT password for authentication

// Root CA Certificate

const char* ca_cert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

/* WiFi and MQTT client Initialization */ 

WiFiClientSecure esp_client;
PubSubClient mqtt_client(esp_client);

#endif

#ifdef BLUETOOTH_DIAGNOSTICS_ENABLED
  BluetoothSerial SerialBT;
#endif

// Circular buffer for storing last N values
float values[BUFFER_SIZE];
int valueIndex = 0;
bool bufferFilled = false;
float lastValidValue = -1;

// Function to calculate average of the buffer
float calculateAverage() {
  float sum = 0;
  int count = bufferFilled ? BUFFER_SIZE : valueIndex;
  
  if (count == 0) return 0;
  
  for (int i = 0; i < count; i++) {
    sum += values[i];
  }
  return sum / count;
}

void Perform_McuReset()
{
  esp_restart();
}

// Add these declarations at the top with other global variables
unsigned long lastMQTTTime = 0;
unsigned long lastThingSpeakTime = 0;
const unsigned long MQTT_INTERVAL = 500;
const unsigned long THINGSPEAK_INTERVAL = 60000;

void setup() 
{
  // Initialize primary Serial for debugging/monitoring
  Serial.begin(115200);
  
  // Initialize UART2 for sensor communication
  Serial2.begin(921600, SERIAL_8N1, RXD2, TXD2);
  
  // Initialize Bluetooth with a simple name
  #ifdef BLUETOOTH_DIAGNOSTICS_ENABLED
    SerialBT.begin("ESP32");  // Using a very simple name for testing
  #endif
  
  Serial.println("Bluetooth Started! Ready to pair...");

  Check_Wifi();

  #ifdef MQTT_DIAGNOSTICS_ENABLED
    MQTT_Init();
  #endif

  #ifdef THINKSPEAK_DIAGNOSTICS_ENABLED
    ThingSpeak_Init();
  #endif
}

String l_MQTT = "";
String l_MQTT_Delta = "";
float distance_cm = 0;
bool IsValidReading = true;

void loop()
{
  if (Serial2.available() >= 16) {  // Wait for complete frame
    uint8_t buffer[16];
    Serial2.readBytes(buffer, 16);
    
    // Verify frame header (should start with 0x57, 0x00)
    if (buffer[0] != 0x57 || buffer[1] != 0x00) {
      // Clear buffer and return if header is invalid
      while(Serial2.available()) Serial2.read();
      return;
    }
    
    // Verify checksum
    uint8_t checksum = 0;
    for (int i = 0; i < 15; i++) {
      checksum += buffer[i];
    }
    checksum &= 0xFF;
    
    if (checksum == buffer[15]) {
      // Extract distance value (3 bytes starting at index 8)
      uint32_t distance_raw = (uint32_t)buffer[8] | 
                            ((uint32_t)buffer[9] << 8) | 
                            ((uint32_t)buffer[10] << 16);
      
      // Convert to centimeters (distance_raw is in millimeters)
      distance_cm = distance_raw / 10.0;  // Convert mm to cm
      
      // Sanity check - ignore unreasonable values
      if (distance_cm >= 0 && distance_cm <= 500) {  // Max range 5 meters
        
        // Check if this is a valid reading (not too different from last value)
        IsValidReading = true;
        if (lastValidValue >= 0) {
          float difference = abs(distance_cm - lastValidValue);
          if (difference > MAX_DIFFERENCE)
          {
            
            l_MQTT_Delta = "PV: " + String(distance_cm, 1) + " cm, Delta: " + String(difference, 1) + " cm";

            Serial.println(l_MQTT_Delta);

            #ifdef BLUETOOTH_DIAGNOSTICS_ENABLED
              SerialBT.println(l_MQTT_Delta);
            #endif
            
            IsValidReading = false;
          }
        }
        
        if (IsValidReading) {
          // Add to circular buffer
          values[valueIndex] = distance_cm;
          valueIndex = (valueIndex + 1) % BUFFER_SIZE;
          if (valueIndex == 0) bufferFilled = true;
          
          // Calculate and send average
          avgDistance = calculateAverage();
          lastValidValue = avgDistance;
          l_MQTT = "PV: " + String(distance_cm, 1) + " cm, Avg: " + String(avgDistance, 1) + " cm";
          Serial.println(l_MQTT);
          
          #ifdef BLUETOOTH_DIAGNOSTICS_ENABLED
            SerialBT.println(l_MQTT);
          #endif
        }
      }
    }
  }
  
  unsigned long currentMillis = millis();
  
  #ifdef MQTT_DIAGNOSTICS_ENABLED
    
    loop_MQTT();

    if (currentMillis - lastMQTTTime >= MQTT_INTERVAL)
    {
      if(IsValidReading == false)
      {
        DEBUG_PRINT_MQTT(l_MQTT_Delta);
      }
      else
      {
        DEBUG_PRINT_MQTT(l_MQTT);
      }
    }
  #endif

  #ifdef THINKSPEAK_DIAGNOSTICS_ENABLED
    if (currentMillis - lastThingSpeakTime >= THINGSPEAK_INTERVAL)
    {
      ThingSpeak_Update_Sl01_CH01();
      lastThingSpeakTime = currentMillis;
    }
  #endif

  delay(100);

} 

/************************** WiFi Handlers - Begin *****************************/

void Check_Wifi()
{
  if(WiFi.status() != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    
    Wifi_TryCount = 0;

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    delay(1000);
    
    while (WiFi.status() != WL_CONNECTED && Wifi_TryCount < WL_RETRY_MAX)
    {
      Serial.print(".");

      delay(5000); 

      Wifi_TryCount++;
    }
    
    if(WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nConnected...");
      Wifi_TryCount = 0;
    }
    else
    {
      Serial.println("\nFailed to Connect, Resetting MCU...");
      Perform_McuReset();
    }
  }
}


/*********************** MQTT Diagnostics - Begin *********************/

#ifdef MQTT_DIAGNOSTICS_ENABLED

void MQTT_Init()
{
  esp_client.setCACert(ca_cert);
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setKeepAlive(60);
  mqtt_client.setCallback(mqttCallback);
  ConnectToMQTT();
}

void loop_MQTT()
{
  ConnectToMQTT();
  mqtt_client.loop();
}

void DEBUG_PRINT_MQTT(String MQTT_String)
{
  mqtt_client.publish(mqtt_topic_Pub, MQTT_String.c_str());
}

void ConnectToMQTT()
{
  /* Using If - For making the connectin as non Blocking... */
  static uint8_t MQTT_Retrv = 10;
  if(!mqtt_client.connected())
  {
    String client_id = "esp32-client-" + String(WiFi.macAddress());
    Serial.printf("Connecting to MQTT Broker as %s...\n", client_id.c_str());

    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("Connected to MQTT broker");
      mqtt_client.subscribe(mqtt_topic_Sub);
      mqtt_client.publish(mqtt_topic_Pub, "CRRC MQTT Begin...");
    }
    else
    {
      Serial.print("Failed to connect to MQTT broker...");
      Serial.print(mqtt_client.state());
      Serial.println(" Delay for 3 seconds...");
      if(MQTT_Retrv > 0)
      {
        delay(3000);
        MQTT_Retrv--;
      }
      else
      {
        Serial.println("\nFailed to Connect to MQTT, Resetting MCU...");
        Perform_McuReset();
      }
      
    }
  }
}


void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("MQTT MSG RX:");
  Serial.println(topic);
  String payloadStr = "";
  for (unsigned int i = 0; i < length; i++)
  {
      payloadStr += (char)payload[i];
  }

  Serial.print("Message: "+String(payloadStr));
}

#endif

/*********************** MQTT Diagnostics - End *********************/

/*********************** ThingSpeak - Begin *************************/

#ifdef THINKSPEAK_DIAGNOSTICS_ENABLED

/* ThingSpeak Init - Begin */
void ThingSpeak_Init(void)
{
  ThingSpeak.begin(TS_client);
}
/* ThingSpeak Init - End */

/* ThingSpeak Update - Begin */
void ThingSpeak_Update_Sl01_CH01()
{
  ThingSpeak.setField(1, (int)millis());
  ThingSpeak.setField(2, distance_cm);
  ThingSpeak.setField(3, avgDistance);
  //ThingSpeak.setField(4, M_Rx_Ctr[3]);
  //ThingSpeak.setField(5, (int16_t)((M_Status_List[0].X_Spare)));
  //ThingSpeak.setField(6, (int16_t)((M_Status_List[1].X_Spare)));
  //ThingSpeak.setField(7, (int16_t)((M_Status_List[2].X_Spare)));
  //ThingSpeak.setField(8, (int16_t)((M_Status_List[3].X_Spare)));
  int x = ThingSpeak.writeFields(myChannelNumber_CH01, myWriteAPIKey_CH01);
  if(x == 200)
  {
    Serial.println("Channel 1 update successful...");
  }
  else
  {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
}

/* ThingSpeak Update - End */

#endif

