#include <Arduino.h>
#include <EEPROM.h>
#include <CAN.h>

#include "utils.h"
#include "main.h"
#include "huawei.h"

// Uncomment to enable debug in this file
// #define ENABLE_DEBUG

#ifdef ENABLE_DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(...)
#endif

namespace Huawei {

// Calibration offset: The physical output is roughly 0.13V lower than the digital reading
const float OUTPUT_VOLTAGE_OFFSET = -0.13f;

bool g_Ready = false;
uint16_t g_UserVoltage = 0x00;
uint16_t g_UserCurrent = 0x00;
float g_UserACLimit = 0.0;           // <--- Added for AC tracking
bool g_UserACLimitEnabled = false;   // <--- Added for AC tracking
float g_Current = 0.0;
float g_CoulombCounter = 0.0;
float g_EnergyCounter = 0.0;
HuaweiInfo g_PSU;

void onRecvCAN(uint32_t msgid, uint8_t *data, uint8_t length)
{
    HuaweiEAddr eaddr = HuaweiEAddr::unpack(msgid);

    switch(eaddr.cmdId)
    {
        case HUAWEI_R48XX_MSG_CURRENT_ID: {
            if(!data[3]) {
                g_Ready = true;
                DEBUG_PRINTLN("PSU ready");
            }
            g_Current = __builtin_bswap16(*(uint16_t *)&data[6]) / 20.0;
            if(!eaddr.fromSrc) {
                g_CoulombCounter += g_Current * 0.377; // every 377ms
                g_EnergyCounter += (g_Current * g_PSU.output_voltage) * 0.377;
            }
        } return;

        case HUAWEI_R48XX_MSG_DATA_ID: {
            uint16_t id = __builtin_bswap16(*(uint16_t *)&data[0]);
            uint32_t val = __builtin_bswap32(*(uint32_t *)&data[4]);
            id &= ~0x3000;

            switch(id)
            {
                case 0x0170: {
                    g_PSU.input_power = val / 1024.0;
                } return;
                case 0x0171: {
                    g_PSU.input_freq = val / 1024.0;
                } return;
                case 0x0172: {
                    g_PSU.input_current = val / 1024.0;
                } return;
                case 0x0173: {
                    g_PSU.output_power = val / 1024.0;
                } return;
                case 0x0174: {
                    g_PSU.efficiency = val / 1024.0;
                } return;
                case 0x0175: {
                    g_PSU.output_voltage = (val / 1024.0) + OUTPUT_VOLTAGE_OFFSET;
                } return;
                case 0x0176: {
                    g_PSU.output_current_max = val / 20.0;
                } return;
                case 0x0178: {
                    g_PSU.input_voltage = val / 1024.0;
                } return;
                case 0x017F: {
                    g_PSU.output_temp = val / 1024.0;
                } return;
                case 0x0180: {
                    g_PSU.input_temp = val / 1024.0;
                } return;
                case 0x0181:
                case 0x0182: {
                    g_PSU.output_current = val / 1024.0;
                } return;
            }
        } return;

        case HUAWEI_R48XX_MSG_INFO_ID: {
            if(data[1] == 1)
                Main::channel()->println("--- HUAWEI R48XX INFO ---");

            switch(data[1]) {
                case 1: {
                    uint32_t val = __builtin_bswap32(*(uint32_t *)&data[4]);
                    uint16_t rated_current = ((val >> 16) & 0x03FF) >> 1;
                    DEBUG_PRINTF("Rated Current: %u\n", rated_current);
                } return;
            }
        } break;

        case HUAWEI_R48XX_MSG_DESC_ID: {
            if(data[1] == 1)
                Main::channel()->println("--- HUAWEI R48XX DESCRIPTION ---");

            Main::channel()->write(&data[2], 6);

            if(!eaddr.count)
                Main::channel()->println();
        } return;
    }

    // Debug
    for(int c = 0; c < Main::NUM_CHANNELS; c++)
    {
        if(Main::g_Debug[c])
        {
            Main::channel(c)->printf("%08X:", msgid);
            for(uint8_t i = 0; i < length; i++)
            {
                Main::channel(c)->printf(" %02X", data[i]);
            }
            Main::channel(c)->println();
        }
    }
}

void every1000ms()
{
    DEBUG_PRINTF("every1000ms, g_Ready=%d\n", g_Ready);
    if(g_Ready)
        sendGetData();

    static bool eepromLoaded = false;
    if(!eepromLoaded && g_Ready) {
        DEBUG_PRINTLN("Loading EEPROM");
        uint32_t magic = 0;
        EEPROM.get(EEPROM_MAGIC_ADDR, magic);
        if(magic == EEPROM_MAGIC_VALUE) {
            float savedVoltage;
            EEPROM.get(EEPROM_VOLTAGE_ADDR, savedVoltage);
            if(!isnan(savedVoltage)) {
                DEBUG_PRINTF("Saved voltage: %.2f\n", savedVoltage);
                if(savedVoltage >= 40.0f && savedVoltage <= 60.0f) {
                    DEBUG_PRINTF("Setting voltage to %.2f\n", savedVoltage);
                    setVoltage(savedVoltage, false);
                }
            } else {
                DEBUG_PRINTLN("Saved voltage invalid (NaN)");
            }

            float savedCurrent;
            EEPROM.get(EEPROM_CURRENT_ADDR, savedCurrent);
            if(!isnan(savedCurrent)) {
                DEBUG_PRINTF("Saved current: %.2f\n", savedCurrent);
                if(savedCurrent >= 0.0f && savedCurrent <= 250.0f) {
                    DEBUG_PRINTF("Setting current to %.2f\n", savedCurrent);
                    setCurrent(savedCurrent, false);
                }
            } else {
                DEBUG_PRINTLN("Saved current invalid (NaN)");
            }
        } else {
            DEBUG_PRINTLN("No valid EEPROM settings found");
        }
            // --- LOAD AC LIMIT ON BOOT ---
            float savedAcLimit;
            EEPROM.get(EEPROM_AC_LIMIT_ADDR, savedAcLimit);
            if(!isnan(savedAcLimit)) {
                DEBUG_PRINTF("Saved AC limit: %.2f\n", savedAcLimit);
                if(savedAcLimit >= 0.0f && savedAcLimit <= 19.0f) {
                    bool enable = (savedAcLimit > 0.0f);
                    // Pass 'false' for perm so we don't unnecessarily re-write EEPROM on boot
                    setInputCurrentLimit(savedAcLimit, enable, false); 
                }
            } else {
                DEBUG_PRINTLN("Saved AC limit invalid (NaN)");
            }
        eepromLoaded = true;
    }

    static uint8_t count10 = 0;
    if(count10 == 10)
    {
        if(g_UserVoltage)
            setVoltageHex(g_UserVoltage, false);
        if(g_UserCurrent)
            setCurrentHex(g_UserCurrent, false);
    }
    count10++;
}

void sendCAN(uint32_t msgid, uint8_t *data, uint8_t length, bool rtr)
{
    DEBUG_PRINTF("sendCAN msgid=0x%08X len=%u rtr=%u\n", msgid, length, rtr);
    CAN.beginExtendedPacket(msgid, -1, rtr);
    CAN.write(data, length);
    CAN.endPacket();
    DEBUG_PRINTLN("CAN packet queued");
}

void setReg(uint8_t reg, uint16_t val)
{
    DEBUG_PRINTF("setReg: reg=0x%02X, val=0x%04X\n", reg, val);
    HuaweiEAddr eaddr;
    eaddr.protoId = HUAWEI_R48XX_PROTOCOL_ID;
    eaddr.addr = 0x00;
    eaddr.cmdId = HUAWEI_R48XX_MSG_CONTROL_ID;
    eaddr.fromSrc = 0x01;
    eaddr.rev = 0x3F;
    eaddr.count = 0x00;
    uint8_t data[8];
    data[0] = 0x01;
    data[1] = reg;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = (val >> 8) & 0xFF;
    data[7] = val & 0xFF;
    DEBUG_PRINTF("Sending data: %02X %02X %02X %02X %02X %02X %02X %02X (msgid=0x%08X)\n",
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], eaddr.pack());
    sendCAN(eaddr.pack(), data, sizeof(data));
}

void setVoltageHex(uint16_t hex, bool perm)
{
    uint8_t reg = perm ? 0x01 : 0x00;

    if(hex < 0xA600)
        hex = 0xA600;
    if(hex > 0xEA00)
        hex = 0xEA00;

    if(perm)
    {
        if(hex < 0xC000)
            hex = 0xC000;
        if(hex > 0xE99A)
            hex = 0xE99A;
    }
    else
        g_UserVoltage = hex;

    setReg(reg, hex);
}

void setVoltage(float u, bool perm)
{
    DEBUG_PRINTF("setVoltage called with u=%.2f, perm=%d\n", u, perm);
    if(perm) {
        EEPROM.put(EEPROM_VOLTAGE_ADDR, u);
        EEPROM.put(EEPROM_MAGIC_ADDR, (uint32_t)EEPROM_MAGIC_VALUE);
        EEPROM.commit();
        DEBUG_PRINTLN("Saved voltage to EEPROM");
    }

    // Restrict inputs to safe hardware bounds
    if(u < 40.0) u = 40.0;
    if(u > 60.0) u = 60.0;

    // To get the actual physical output to match 'u', we must tell the 
    // internal DAC to aim slightly higher to overcome the hardware voltage drop.
    // Subtracting a negative offset (e.g. 55.0 - (-0.13)) adds it to the target.
    uint16_t hex = (u - OUTPUT_VOLTAGE_OFFSET) * 1024.0;
    
    DEBUG_PRINTF("Calculated hex: %d\n", hex);
    setVoltageHex(hex, perm);
}

void setCurrentHex(uint16_t hex, bool perm)
{
    uint8_t reg = perm ? 0x04 : 0x03;

    if(hex > 0x0499)
        hex = 0x0499;

    if(!perm)
        g_UserCurrent = hex;

    setReg(reg, hex);
}

void setCurrent(float i, bool perm)
{
    if(perm) {
        EEPROM.put(EEPROM_CURRENT_ADDR, i);
        EEPROM.put(EEPROM_MAGIC_ADDR, (uint32_t)EEPROM_MAGIC_VALUE);
        EEPROM.commit();
        DEBUG_PRINTLN("Saved current to EEPROM");
    }

    uint16_t hex = i * 20.0;

    setCurrentHex(hex, perm);
}

// Fixed function signature right here!
void setInputCurrentLimit(float amps, bool enable, bool perm)
{
    // The R4850G2 has a hardware maximum of ~17A-19A on the AC side.
    if(amps > 19.0) amps = 19.0;
    if(amps < 0.0) amps = 0.0;

    // --- TRACK STATE FOR STATUS PRINT ---
    g_UserACLimit = amps;
    g_UserACLimitEnabled = enable;

    // --- EEPROM SAVING LOGIC ---
    if(perm) {
        // If disabled, we save 0.0 so it stays disabled on next boot
        float valueToSave = enable ? amps : 0.0f; 
        EEPROM.put(EEPROM_AC_LIMIT_ADDR, valueToSave);
        EEPROM.put(EEPROM_MAGIC_ADDR, (uint32_t)EEPROM_MAGIC_VALUE);
        EEPROM.commit();
        DEBUG_PRINTLN("Saved AC limit to EEPROM");
    }

    DEBUG_PRINTF("Setting AC Input Limit: %.2fA, Enabled: %d\n", amps, enable);

    HuaweiEAddr eaddr;
    eaddr.protoId = HUAWEI_R48XX_PROTOCOL_ID;
    eaddr.addr = 0x00;
    eaddr.cmdId = HUAWEI_R48XX_MSG_CONTROL_ID;
    eaddr.fromSrc = 0x01;
    eaddr.rev = 0x3F;
    eaddr.count = 0x00;

    // Convert Amps to the 32-bit hex value expected by the protocol
    uint32_t val = (uint32_t)(amps * 1024.0);
    
    uint8_t data[8];
    data[0] = 0x01;
    data[1] = 0x09; // Register 0x09: AC Input Current Limit
    data[2] = 0x00;
    data[3] = enable ? 0x01 : 0x00; // Byte 3 acts as the ON/OFF switch
    data[4] = (val >> 24) & 0xFF;
    data[5] = (val >> 16) & 0xFF;
    data[6] = (val >> 8) & 0xFF;
    data[7] = val & 0xFF;

    sendCAN(eaddr.pack(), data, sizeof(data));
}

void sendGetData()
{
    HuaweiEAddr eaddr;
    eaddr.protoId = HUAWEI_R48XX_PROTOCOL_ID;
    eaddr.addr = 0x00;
    eaddr.cmdId = HUAWEI_R48XX_MSG_DATA_ID;
    eaddr.fromSrc = 0x01;
    eaddr.rev = 0x3F;
    eaddr.count = 0x00;
    uint8_t data[8] = {0x00};
    sendCAN(eaddr.pack(), data, sizeof(data));
}

void sendGetInfo()
{
    HuaweiEAddr eaddr;
    eaddr.protoId = HUAWEI_R48XX_PROTOCOL_ID;
    eaddr.addr = 0x00;
    eaddr.cmdId = HUAWEI_R48XX_MSG_INFO_ID;
    eaddr.fromSrc = 0x01;
    eaddr.rev = 0x3F;
    eaddr.count = 0x00;
    uint8_t data[8] = {0x00};
    sendCAN(eaddr.pack(), data, sizeof(data));
}

void sendGetDescription()
{
    HuaweiEAddr eaddr;
    eaddr.protoId = HUAWEI_R48XX_PROTOCOL_ID;
    eaddr.addr = 0x00;
    eaddr.cmdId = HUAWEI_R48XX_MSG_DESC_ID;
    eaddr.fromSrc = 0x01;
    eaddr.rev = 0x3F;
    eaddr.count = 0x00;
    uint8_t data[8] = {0x00};
    sendCAN(eaddr.pack(), data, sizeof(data));
}

HuaweiEAddr HuaweiEAddr::unpack(uint32_t val)
{
    HuaweiEAddr s;
    s.protoId = (val>>23) & 0x3F;
    s.addr = (val>>16) & 0x7F;
    s.cmdId = (val>>8) & 0xFF;
    s.fromSrc = (val>>7) & 0x01;
    s.rev = (val>>1) & 0x3F;
    s.count = val & 0x01;
    return s;
}

uint32_t HuaweiEAddr::pack()
{
    return protoId << 23 |
           addr << 16 |
           cmdId << 8 |
           fromSrc << 7 |
           rev << 1 |
           count;
}

}