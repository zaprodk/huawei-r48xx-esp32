#define HEARTBEAT_LED 2

#include <Arduino.h>
#include <EEPROM.h>
#include <CAN.h>
#include <BluetoothSerial.h>

#include "huawei.h"
#include "commands.h"
#include "main.h"
#include "secrets.h"


// --- FEATURE FLAGS ---
// Uncomment the next line to ENABLE Wi-Fi and OTA updates.
// #define ENABLE_WIFI

// Uncomment the next line to ENABLE Serial Debug output via USB.
// Leave commented out for production to save CPU cycles and RAM.
// #define ENABLE_DEBUG

// --- DEBUG MACROS ---
#ifdef ENABLE_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

#ifdef ENABLE_WIFI
  #include <WiFi.h>
  #include <ArduinoOTA.h>
  
  WiFiServer server(23);
  WiFiClient serverClient;
  
  const char g_WIFI_SSID[] = SECRET_WIFI_SSID;
  const char g_WIFI_Passphrase[] = SECRET_WIFI_PASS;
#else
  #include <WiFi.h> // Still needed to send the WIFI_OFF command
#endif

BluetoothSerial SerialBT;

namespace Main
{

int g_CurrentChannel;
bool g_Debug[NUM_CHANNELS];
char g_SerialBuffer[NUM_CHANNELS][255];
int g_SerialBufferPos[NUM_CHANNELS];
unsigned long g_Time1000;

// --- APPLICATION SECURITY FLAG ---
bool g_BT_Authenticated = false;

} // Temporarily closing namespace to define the BT callback

// --- BLUETOOTH CONNECTION CALLBACK ---
void bt_connection_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
        DEBUG_PRINTLN("Bluetooth Client Connected. System Locked.");
        Main::g_BT_Authenticated = false; // Lock the system on new connection
        SerialBT.println("SYSTEM LOCKED. Enter 4-digit PIN:");
    } else if (event == ESP_SPP_CLOSE_EVT) {
        DEBUG_PRINTLN("Bluetooth Client Disconnected.");
        Main::g_BT_Authenticated = false; // Lock the system on disconnect
    }
}

namespace Main 
{ // Re-opening namespace

void onCANReceive(int packetSize)
{
    if(!CAN.packetExtended())
        return;
    if(CAN.packetRtr())
        return;

    uint32_t msgid = CAN.packetId();

    uint8_t data[packetSize];
    CAN.readBytes(data, sizeof(data));

    Huawei::onRecvCAN(msgid, data, packetSize);
}

void init()
{
    Serial.begin(115200); // We keep this active so you can still send commands over USB
    while(!Serial);
    DEBUG_PRINTLN("BOOTED!");

    // PSU enable pin
    pinMode(POWER_EN_GPIO, OUTPUT_OPEN_DRAIN);
    digitalWrite(POWER_EN_GPIO, 0); // Default = ON

    // Heartbeat LED setup
    pinMode(HEARTBEAT_LED, OUTPUT);
    digitalWrite(HEARTBEAT_LED, HIGH); // Active low: HIGH = OFF

#ifdef ENABLE_WIFI
    // --- WI-FI & OTA SETUP ---
    WiFi.setHostname("Huawei-R4850G2");
    if(!WiFi.begin(g_WIFI_SSID, g_WIFI_Passphrase))
        DEBUG_PRINTLN("WiFi config error!");
    else {
        WiFi.setAutoConnect(true);
    }

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";
        DEBUG_PRINTLN("Start updating " + type);
    })
    .onEnd([]() {
        DEBUG_PRINTLN("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
        DEBUG_PRINTF("Error[%u]: ", error);
    });

    ArduinoOTA.begin();

    server.begin();
    server.setNoDelay(true);
#else
    // Explicitly turn OFF the Wi-Fi radio
    WiFi.mode(WIFI_OFF); 
#endif

    // --- CLEAN BLUETOOTH START ---
    SerialBT.register_callback(bt_connection_callback);
    SerialBT.begin("Huawei-R4850G2"); 

    if(!CAN.begin(125E3)) {
        DEBUG_PRINTLN("Starting CAN failed!");
        while(1);
    }

    EEPROM.begin(512);
}

Stream* channel(int num)
{
    if(num == -1)
        num = g_CurrentChannel;

    if(num == BTSERIAL && SerialBT.hasClient())
        return &SerialBT;
        
#ifdef ENABLE_WIFI
    else if(num == TCPSERIAL && serverClient)
        return &serverClient;
#endif

    return &Serial;
}

void loop()
{
    int packetSize = CAN.parsePacket();
    if(packetSize)
        onCANReceive(packetSize);

#ifdef ENABLE_WIFI
    ArduinoOTA.handle();

    if(server.hasClient())
    {
        if(serverClient) // disconnect current client if any
            serverClient.stop();
        serverClient = server.available();
    }
    if(!serverClient)
        serverClient.stop();
#endif

    for(int i = 0; i < NUM_CHANNELS; i++)
    {
        while(channel(i)->available())
        {
            g_CurrentChannel = i;
            int c = channel(i)->read();
            
            if(c == '\r' || c == '\n' || g_SerialBufferPos[i] >= (sizeof(*g_SerialBuffer) - 1))
            {
                g_SerialBuffer[i][g_SerialBufferPos[i]] = 0; 

                if(g_SerialBufferPos[i] > 0)
                {
                    // --- APPLICATION LAYER SECURITY LOGIC ---
                    if (i == BTSERIAL && !g_BT_Authenticated) 
                    {
                        if (strcmp(g_SerialBuffer[i], SECRET_BT_PIN) == 0)
                        {
                            g_BT_Authenticated = true;
                            SerialBT.println("Access Granted. System Ready.");
                            DEBUG_PRINTLN("Bluetooth client authenticated successfully.");
                        } 
                        else 
                        {
                            SerialBT.println("ERROR: Unauthorized. Enter 4-digit PIN:");
                            DEBUG_PRINTLN("Bluetooth client failed authentication.");
                        }
                    } 
                    else 
                    {
                        Commands::parseLine(g_SerialBuffer[i]);
                    }
                }

                g_SerialBufferPos[i] = 0;
                continue;
            }
            g_SerialBuffer[i][g_SerialBufferPos[i]] = c;
            ++g_SerialBufferPos[i];
        }
    }

    if((millis() - g_Time1000) > 1000)
    {
        Huawei::every1000ms();
        g_Time1000 = millis();
        // --- HEARTBEAT LED TOGGLE ---
        static bool led_state = false;
        led_state = !led_state; // Flip the state
        // Active low: When led_state is true, output LOW (ON). Otherwise output HIGH (OFF).
        digitalWrite(HEARTBEAT_LED, led_state ? LOW : HIGH);
    }
}

} // End namespace Main

void setup()
{
    Main::init();
}

void loop()
{
    Main::loop();
}