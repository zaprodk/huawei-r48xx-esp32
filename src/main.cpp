#include <Arduino.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <CAN.h>
#include <BluetoothSerial.h>

#include "huawei.h"
#include "commands.h"
#include "main.h"
#include "secrets.h"

WiFiServer server(23);
WiFiClient serverClient;

BluetoothSerial SerialBT;

const char g_WIFI_SSID[] = SECRET_WIFI_SSID;
const char g_WIFI_Passphrase[] = SECRET_WIFI_PASS;

namespace Main
{

int g_CurrentChannel;
bool g_Debug[NUM_CHANNELS];
char g_SerialBuffer[NUM_CHANNELS][255];
int g_SerialBufferPos[NUM_CHANNELS];
unsigned long g_Time1000;

// --- APPLICATION SECURITY FLAG ---
// This tracks if the current Bluetooth connection has entered the PIN
bool g_BT_Authenticated = false;

} // Temporarily closing namespace to define the BT callback

// --- BLUETOOTH CONNECTION CALLBACK ---
// This resets the security lock whenever a device connects or disconnects.
// It ensures that if you disconnect, the next person is locked out.
void bt_connection_callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    if (event == ESP_SPP_SRV_OPEN_EVT) {
        Serial.println("Bluetooth Client Connected. System Locked.");
        Main::g_BT_Authenticated = false; // Lock the system on new connection
        SerialBT.println("SYSTEM LOCKED. Enter 4-digit PIN:");
    } else if (event == ESP_SPP_CLOSE_EVT) {
        Serial.println("Bluetooth Client Disconnected.");
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
    Serial.begin(115200);
    while(!Serial);
    Serial.println("BOOTED!");

    // PSU enable pin
    pinMode(POWER_EN_GPIO, OUTPUT_OPEN_DRAIN);
    digitalWrite(POWER_EN_GPIO, 0); // Default = ON

    WiFi.setHostname("Huawei-R4850G2");
    if(!WiFi.begin(g_WIFI_SSID, g_WIFI_Passphrase))
        Serial.println("WiFi config error!");
    else {
        WiFi.setAutoConnect(true);
    }

    // --- CLEAN BLUETOOTH START ---
    // Register the callback to handle connect/disconnect events
    SerialBT.register_callback(bt_connection_callback);
    // Start BT normally ("Just Works" mode)
    SerialBT.begin("Huawei-R4850G2"); 

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";
        Serial.println("Start updating " + type);
    })
    .onEnd([]() {
        Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
    });

    ArduinoOTA.begin();

    server.begin();
    server.setNoDelay(true);

    if(!CAN.begin(125E3)) {
        Serial.println("Starting CAN failed!");
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
    else if(num == TCPSERIAL && serverClient)
        return &serverClient;

    return &Serial;
}

void loop()
{
    int packetSize = CAN.parsePacket();
    if(packetSize)
        onCANReceive(packetSize);

    ArduinoOTA.handle();

    if(server.hasClient())
    {
        if(serverClient) // disconnect current client if any
            serverClient.stop();
        serverClient = server.available();
    }
    if(!serverClient)
        serverClient.stop();

    for(int i = 0; i < NUM_CHANNELS; i++)
    {
        while(channel(i)->available())
        {
            g_CurrentChannel = i;
            int c = channel(i)->read();
            
            // Check for end of line OR buffer full
            if(c == '\r' || c == '\n' || g_SerialBufferPos[i] >= (sizeof(*g_SerialBuffer) - 1))
            {
                g_SerialBuffer[i][g_SerialBufferPos[i]] = 0; // Null terminate the string

                if(g_SerialBufferPos[i] > 0)
                {
                    // --- APPLICATION LAYER SECURITY LOGIC ---
                    if (i == BTSERIAL && !g_BT_Authenticated) 
                    {
                        // Client is on Bluetooth and NOT authenticated yet
                        if (strcmp(g_SerialBuffer[i], SECRET_BT_PIN) == 0)
                        {
                            g_BT_Authenticated = true;
                            SerialBT.println("Access Granted. System Ready.");
                            Serial.println("Bluetooth client authenticated successfully.");
                        } 
                        else 
                        {
                            SerialBT.println("ERROR: Unauthorized. Enter 4-digit PIN:");
                            Serial.println("Bluetooth client failed authentication.");
                        }
                    } 
                    else 
                    {
                        // If it's a local Serial command, TCP, OR an authenticated Bluetooth command, parse it
                        Commands::parseLine(g_SerialBuffer[i]);
                    }
                    // --- END SECURITY LOGIC ---
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