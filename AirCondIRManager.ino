/*
 * This is an Arduino/ESP8266 project to control your air conditioner automatically.
 * Source code: https://github.com/AlexeySofree/AirCondIRManager
 * Author: Alexey Tsarev, Tsarev.Alexey at gmail.com
 */

//#define ESP8266

#include "EEPROMer.h"
#include <DallasTemperature.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#ifdef ESP8266
// ESP8266 libraries
#include <IRremoteESP8266.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

#else
// Arduino libraries
#include <IRremote.h>
/*
IRremoteInt.h - Change to use #define IR_USE_TIMER1
                Change to minimal for your IR transmitter value at the #define RAWBUF
                My case - #define RAWBUF 70
IRremote.h - Turn off all #define SEND_*, #define DECODE_*
             Turn on only yours DECODE_*
             My case - turned on DECODE_RC5, DECODE_SAMSUNG
*/
#endif


// Cfg
#ifdef ESP8266
// ESP8266
#define configPinLED        16
#define configPinTSOP       14
#define configPinTempSensor 0
#define configPinBeeper     15
/*
         * * * Wemos D1 * * *
Real number  Title on plate  Description
03               RX          Firmware upload failed when an active load at the pin
16               D0          Not started when an active load at the pin
05           SLC/D1,  SLC
04           SDA/D2,  SDA
00               D3
02               D4          Internal LED
14               D5
12               D6, MISO/D6
13               D7, MOSI/D7
15            SS/D8
*/

#define configOnConfigAPName "AirCondManager"
#define configOnConfigAPTimeOutSeconds 120
#define configHTTPPort 80
#else
// Arduino
#define configPinLED        3
#define configPinTSOP       4
#define configPinTempSensor 5
#define configPinBeeper     6
#endif

#define configTempSensorResolution 12 // 9 - 12
#define configMaxWaitSetupModeMillis 5000
#define configScreenDelayOnKeyPressMillis 250
#define configLEDCarrierFreq 38000
#define configSerialSpeed 115200
#define configIRCmdRepeats 5
#define configIRCmdDelayMillis 2000
#define configDisplayTempMin 10
#define configDisplayTempMax 40
#define configDisplayDeltaTempMin 1
#define configDisplayDeltaTempMax 9
unsigned int configBuzzerBeepHz = 4000;
unsigned int configBuzzerBeepMillis = 100;
unsigned int configBuzzerAlarmHz[] = {1000, 1500, 1200, 1300};
unsigned int configBuzzerAlarmMillis[] = {200, 200, 200, 200};
// End Cfg

#define capDataMaxLen 70 // My Haier air cond has 230, but Electrolux air cond has 68
#define maxWaitCapDataMicros 50000 // No IR signal time to stop capture

struct config {
    DeviceAddress tempSensorDeviceAddress;
    byte tempSensorResolution;

    byte tempMin;
    unsigned long buttonMinUp;
    unsigned long buttonMinDown;

    byte tempMax;
    unsigned long buttonMaxUp;
    unsigned long buttonMaxDown;

    byte tempDeltaMax;
    unsigned long buttonDeltaMaxUp;
    unsigned long buttonDeltaMaxDown;

    byte commandOnLen;
    byte commandOnDivider;
    byte commandOffLen;
    byte commandOffDivider;
};

// EEPROM data
config theConfig;
byte commandOn[capDataMaxLen];
byte commandOff[capDataMaxLen];
// End

byte t1, t2, tempDeltaMaxValue, buzzerAlarmMelodyLen, curBuzzerAlarm, status;
bool b, b1, gotTempFlag = false, tempGrowFlag, setupModeFlag = false, screenUpdateFlag, screenDelayFlag, tempMinActiveFlag = false, tempMaxActiveFlag = false, beepFlag = false, alarmFlag = false, alarmBuzzerFlag, sendingCmdsFlag = false;
char strBuf[32], strBuf2[32];
unsigned int i, i1, i2, tempSensorWaitDataDelayMillis, startAddressConfig, startAddressConfigCommandOn, startAddressConfigCommandOff, EEPROMTotalSize;
unsigned long lastTempRequestMillis, lastSetupModeButtonPressMillis, lastScreenDelayMillis, lastBuzzerAlarmMillis, lastIRSentMillis, lastBeepMillis;
float curTemp, prevTemp = -274, tmpTemp;
#ifdef ESP8266
bool WLANConnectedFlag = false;
unsigned long addSec = 0, prevMillis;
unsigned int addMillis = 0;
#endif

// Arduino, 4 pin OLED
//U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0,/*SCL*/ 11,/*SDA*/ 10,/*cs*/ U8X8_PIN_NONE,/*D/C*/ 8,/*RST*/ 9);
// ESP8266, 4 pin OLED
//U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0,/*SCL*/ SCL,/*SDA*/ SDA,/*cs*/ U8X8_PIN_NONE,/*D/C*/ 13,/*RST*/ 12);
// ESP8266, 2 pin OLED
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);

DeviceAddress tempDeviceAddress;
OneWire oneWire(configPinTempSensor);
DallasTemperature tempSensor(&oneWire);
IRrecv IRRecv(configPinTSOP);
decode_results IRResults;
#ifdef ESP8266
ESP8266WebServer server(configHTTPPort);
#endif

// Strings at PROGMEM
const char strStart[] PROGMEM = "Start";
const char strInitialize[] PROGMEM = "Initialize";
const char strReadingEEPROM[] PROGMEM = "Reading EEPROM";
const char strUpdatingEEPROM[] PROGMEM = "Updating EEPROM ";
const char strArrayLen[] PROGMEM = "Array len";
const char strCapturedRaw[] PROGMEM = "Captured raw";
const char strFoundMax[] PROGMEM = "Found max";
const char strDividedBy[] PROGMEM = "Divided by";
const char strMultipliedBy[] PROGMEM = "Multiplied by";
const char strFoundTempSensor[] PROGMEM = "Found temperature sensor(s)";
const char strTemperature[] PROGMEM = "Temperature ";
const char strSensor[] PROGMEM = "sensor ";
const char strNotFound[] PROGMEM = "not found";
const char strParasitePower[] PROGMEM = "Parasite power";
const char strON[] PROGMEM = "ON";
const char strOFF[] PROGMEM = "OFF";
const char strNoAddressForSensor[] PROGMEM = "No address for sensor: 0";
const char strDevAddress[] PROGMEM = "Device address";
const char strEEPROMDeviceAddress[] PROGMEM = "EEPROM DeviceAddress";
const char strEEPROMSensorResolution[] PROGMEM = "EEPROM SensorResolution";
const char strEEPROMDifferentData[] PROGMEM = "EEPROM has different data";
const char strDone[] PROGMEM = "done";
const char strGotTemp[] PROGMEM = "Got temperature";
const char strGettingTemp[] PROGMEM = "Getting temp";
const char strGotIR[] PROGMEM = "Got IR";
const char strPrintingScreen[] PROGMEM = "Printing screen";
const char strIgnoredIR[] PROGMEM = "Ignored IR";
const char strBeep[] PROGMEM = "Beep";
const char strSendingIR[] PROGMEM = "Sending IR";
const char strStartAddress[] PROGMEM = "Start address";
const char strTotalSize[] PROGMEM = "Total size";
const char strSetupMode[] PROGMEM = "SetupMode";
const char strPressButton[] PROGMEM = "PressButton";
const char strCommaSeparator[] PROGMEM = ", ";
const char strDotSeparator[] PROGMEM = ". ";
const char str3DotSeparator[] PROGMEM = "... ";
const char strDivideSeparator[] PROGMEM = " / ";
const char strColonSeparator[] PROGMEM = ": ";
const char strSetupButton_0[] PROGMEM = "MinUp";
const char strSetupButton_1[] PROGMEM = "MinDn";
const char strSetupButton_2[] PROGMEM = "MaxUp";
const char strSetupButton_3[] PROGMEM = "MaxDn";
const char strSetupButton_4[] PROGMEM = "DMaxU";
const char strSetupButton_5[] PROGMEM = "DMaxD";
const char strSetupButton_6[] PROGMEM = "ON";
const char strSetupButton_7[] PROGMEM = "OFF";
const char *const strSetupButtons[] PROGMEM = {strSetupButton_0, strSetupButton_1, strSetupButton_2, strSetupButton_3,
                                               strSetupButton_4, strSetupButton_5, strSetupButton_6, strSetupButton_7};
#ifdef ESP8266
const char strWaiting[] PROGMEM = "Waiting";
const char strWiFi[] PROGMEM = "WiFi";
const char strConnectedTo[] PROGMEM = "Connected to";
const char strAPConfig[] PROGMEM = "AP config:";
const char strTO[] PROGMEM = "T/Out";
const char strTimedOut[] PROGMEM = "Timed out";
const char strWiFiStatusChangedTo[] PROGMEM = "WiFi connection status changed to";
const char strStartingHTTPServer[] PROGMEM = "Starting HTTP server";
const char strTextPlain[] PROGMEM = "text/plain";
const char strBusy[] PROGMEM = "Busy";
#endif
// End Strings

// Functions
#define lg(str)         Serial.print(str)
#define log(str)        Serial.println(str)
#define lgFmt(str, fmt) Serial.print(str,fmt)
#define setFont1        u8g2.setFont(u8g2_font_courB14_tr)
#define setFont2        u8g2.setFont(u8g2_font_inb19_mn)


void lgPM(const char s[]) {
    unsigned int i, len = strlen_P(s);
    char str;

    for (i = 0; i < len; i++) {
        str = (char) pgm_read_byte_near(s + i);
        lg(str);
    }
}


void logPM(const char s[]) {
    lgPM(s);
    log();
}


bool isElapsedTimeFromTS(unsigned long start, unsigned long elapsed, bool isMicros = false) {
    unsigned long cur;
    unsigned long delta;

    if (isMicros)
        cur = micros();
    else
        cur = millis();

    if (cur >= start)
        delta = cur - start;
    else
        delta = 0xFFFFFFFF - start + cur + 1;

    return delta > elapsed;
}


void sendIR(byte *raw, byte rawLen, byte pinLED, unsigned int carrierFreq, byte multiplier = 1) {
    byte startPos, i;
    bool turnedOnFlag = true, carrierOnFlag;
    unsigned long startTSMicros;
    unsigned int delayMicros;
    double halfPeroidMicros, carrierWaitMicros;

    if (raw[0] == 0)
        startPos = 1;
    else
        startPos = 0;

    halfPeroidMicros = 500000 / carrierFreq;
    startTSMicros = micros();

    for (i = startPos; i < rawLen; i++) {
        if (multiplier == 1)
            delayMicros = raw[i];
        else
            delayMicros = raw[i] * multiplier;

        carrierWaitMicros = 0;
        carrierOnFlag = true;

        while (!isElapsedTimeFromTS(startTSMicros, delayMicros, true)) {
            if (turnedOnFlag) {
                if (carrierOnFlag)
                    digitalWrite(pinLED, HIGH);
                else
                    digitalWrite(pinLED, LOW);

                carrierWaitMicros += halfPeroidMicros;

                while (!isElapsedTimeFromTS(startTSMicros, carrierWaitMicros, true));

                carrierOnFlag = !carrierOnFlag;
            }
        }

        startTSMicros += delayMicros;
        turnedOnFlag = !turnedOnFlag;

        if (turnedOnFlag)
            noTone(pinLED);
    }

    digitalWrite(pinLED, LOW);
}


void printDeviceAddress(DeviceAddress deviceAddress) {
    for (byte i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16)
            lg("0");

        lgFmt(deviceAddress[i], HEX);
    }

    log();
}


void captureRawIRData(byte cap1[], byte cap2[], byte output[], byte &outputLen, byte &outputDivider) {
    bool waitBit, curBit, maxWaitMillisFlag = false;
    unsigned int i = 0, j, max = 0;
    unsigned long lastTS = 0, curMicros;

    waitBit = !digitalRead(configPinTSOP);

    while (i < capDataMaxLen) {
        do {
            curBit = digitalRead(configPinTSOP);

            if (i == 0) {
#ifdef ESP8266
                ESP8266Tasks();
#endif
            } else if (isElapsedTimeFromTS(lastTS, maxWaitCapDataMicros, true)) {
                maxWaitMillisFlag = true;
                break;
            }
        } while (curBit != waitBit);

        if (maxWaitMillisFlag)
            break;

        curMicros = micros();
        j = curMicros - lastTS;
        cap1[i] = highByte(j);
        cap2[i] = lowByte(j);
        lastTS = curMicros;
        waitBit = !waitBit;

        i++;
    }

    cap1[0] = 0;
    cap2[0] = 0;
    outputLen = i;

    lgPM(strArrayLen);
    lgPM(strColonSeparator);
    log(i);

    lgPM(strCapturedRaw);
    logPM(strColonSeparator);
    for (j = 1; j < i; j++) {
        lg(word(cap1[j], cap2[j]));
        lgPM(strCommaSeparator);

        if (word(cap1[j], cap2[j]) > max)
            max = word(cap1[j], cap2[j]);
    }
    log();

    lgPM(strFoundMax);
    lgPM(strColonSeparator);
    log(max);

    outputDivider = highByte(max) + 1;

    lgPM(strDividedBy);
    lgPM(strColonSeparator);
    log(outputDivider);
    for (j = 1; j < i; j++) {
        output[j] = word(cap1[j], cap2[j]) / outputDivider;
        lg(output[j]);
        lgPM(strCommaSeparator);
    }
    log();

    lgPM(strMultipliedBy);
    lgPM(strColonSeparator);
    log(outputDivider);
    for (j = 1; j < i; j++) {
        lg(output[j] * outputDivider);
        lgPM(strCommaSeparator);
    }
    log();
}


void requestTemp() {
    tempSensor.requestTemperatures();
    lastTempRequestMillis = millis();
}


void findTempSensor() {
    while (true) {
        tempSensor.begin();
        i = tempSensor.getDeviceCount();
        lgPM(strFoundTempSensor);
        lgPM(strColonSeparator);
        log(i);

        if (i == 0) {
            lgPM(strTemperature);
            lgPM(strSensor);
            logPM(strNotFound);

            u8g2.firstPage();
            setFont1;
            do {
                strcpy_P(strBuf, strTemperature);
                u8g2.drawStr(0, 12, strBuf);

                strcpy_P(strBuf, strSensor);
                u8g2.drawStr(0, 36, strBuf);

                strcpy_P(strBuf, strNotFound);
                u8g2.drawStr(0, 60, strBuf);
#ifdef ESP8266
                ESP8266Tasks();
#endif
            } while (u8g2.nextPage());

            continue;
        }

        lgPM(strParasitePower);
        lgPM(strColonSeparator);
        if (tempSensor.isParasitePowerMode())
            logPM(strON);
        else
            logPM(strOFF);

        if (!tempSensor.getAddress(tempDeviceAddress, 0)) {
            logPM(strNoAddressForSensor);
            continue;
        }

        lgPM(strDevAddress);
        lgPM(strColonSeparator);
        printDeviceAddress(tempDeviceAddress);

        break;
    }
}


void calculateEEPROMStartAddresses() {
    EEPROMTotalSize = 0;
    startAddressConfig = EEPROMTotalSize;

    lgPM(strStartAddress);
    lgPM(strColonSeparator);
    log(startAddressConfig);

    EEPROMTotalSize += sizeof(theConfig);
    startAddressConfigCommandOn = EEPROMTotalSize;

    lgPM(strStartAddress);
    lgPM(strColonSeparator);
    log(startAddressConfigCommandOn);

    EEPROMTotalSize += sizeof(commandOn);
    startAddressConfigCommandOff = EEPROMTotalSize;

    lgPM(strStartAddress);
    lgPM(strColonSeparator);
    log(startAddressConfigCommandOff);

    EEPROMTotalSize += sizeof(commandOff);

    lgPM(strTotalSize);
    lgPM(strColonSeparator);
    log(EEPROMTotalSize);
}


void loadConfig() {
    lgPM(strReadingEEPROM);
    lgPM(str3DotSeparator);

    EEPROMReader(startAddressConfig, theConfig, EEPROMTotalSize);
    EEPROMReader(startAddressConfigCommandOn, commandOn, EEPROMTotalSize);
    EEPROMReader(startAddressConfigCommandOff, commandOff, EEPROMTotalSize);

    logPM(strDone);
}


void updateConfig(byte configBlock = 0) {
    lgPM(strUpdatingEEPROM);
    lg(configBlock);
    lgPM(str3DotSeparator);

    if ((configBlock == 0) || (configBlock == 1))
        EEPROMUpdater(startAddressConfig, theConfig, EEPROMTotalSize);

    if ((configBlock == 0) || (configBlock == 2))
        EEPROMUpdater(startAddressConfigCommandOn, commandOn, EEPROMTotalSize);

    if ((configBlock == 0) || (configBlock == 3))
        EEPROMUpdater(startAddressConfigCommandOff, commandOff, EEPROMTotalSize);

    logPM(strDone);
}


void beep(bool newLine = true) {
    lgPM(strBeep);

    tone(configPinBeeper, configBuzzerBeepHz, configBuzzerBeepMillis);
    lastBeepMillis = millis();
    beepFlag = true;

    if (newLine)
        log();
}


void readTempSensorData() {
    lgPM(strGettingTemp);
    lgPM(str3DotSeparator);

    if (tempSensor.getTempCByIndex(0) != DEVICE_DISCONNECTED_C)
        curTemp = tempSensor.getTempCByIndex(0);

    lgPM(strDone);
    lgPM(strDotSeparator);
    lgPM(strGotTemp);
    lgPM(strColonSeparator);
    lgFmt(curTemp, 4);

    if (!gotTempFlag) {
        gotTempFlag = true;
        log();

#ifdef ESP8266
        if (WiFi.status() == WL_CONNECTED)
            startHTTP();
        else
            setupWLAN();
#endif

        lgPM(strSetupMode);
        lgPM(strColonSeparator);
        lg(i1);
        lgPM(strDivideSeparator);
        lg(i);
        lgPM(strColonSeparator);

        IRRecv.enableIRIn();

        if (i1 == i) {
            lgPM(strON);
            setupModeFlag = true;
            lastSetupModeButtonPressMillis = millis();
            i1 = 0;
            b1 = false;
            log();
            beep(false);
            return;
        } else {
            lgPM(strOFF);
        }
    }

    if (prevTemp != curTemp) {
        tempGrowFlag = curTemp > prevTemp;
        prevTemp = curTemp;
        screenUpdateFlag = true;
    }
}


void correctConfigValues() {
    if (theConfig.tempMin > configDisplayTempMax)
        theConfig.tempMin = configDisplayTempMax;

    if (theConfig.tempMin < configDisplayTempMin)
        theConfig.tempMin = configDisplayTempMin;

    if (theConfig.tempMax > configDisplayTempMax)
        theConfig.tempMax = configDisplayTempMax;

    if (theConfig.tempMax < configDisplayTempMin)
        theConfig.tempMax = configDisplayTempMin;

    if (theConfig.tempDeltaMax > configDisplayDeltaTempMax)
        theConfig.tempDeltaMax = configDisplayDeltaTempMax;

    if (theConfig.tempDeltaMax < configDisplayDeltaTempMin)
        theConfig.tempDeltaMax = configDisplayDeltaTempMin;

    if (theConfig.tempMin >= theConfig.tempMax)
        theConfig.tempMin = theConfig.tempMax - 1;

    if (theConfig.commandOnLen > capDataMaxLen)
        theConfig.commandOnLen = capDataMaxLen;

    if (theConfig.commandOffLen > capDataMaxLen)
        theConfig.commandOffLen = capDataMaxLen;
}


void ignoreIRResults() {
    if (IRRecv.decode(&IRResults)) {
        lgPM(strDotSeparator);
        lgPM(strIgnoredIR);
        lgPM(strColonSeparator);
        lgFmt(IRResults.value, HEX);
        lgPM(strCommaSeparator);
        lg(IRResults.decode_type);

        IRRecv.resume();
    }
}


void printAndSendCmds(const char strCmd[], byte cmd[capDataMaxLen], byte cmdLen, byte cmdDivider, bool &on, bool &off,
                      byte repeats) {
    byte i, j, len = strlen_P(strCmd);

    sendingCmdsFlag = true;

    for (i = 0; i < repeats; i++) {
        lastIRSentMillis = millis();

        u8g2.firstPage();
        setFont1;
        do {
            dtostrf(curTemp, 2, 2, strBuf);
            sprintf(strBuf2, "%6s", strBuf);
            u8g2.drawStr(35, 15, strBuf2);

            sprintf(strBuf, "%i/%i", i + 1, repeats);
            u8g2.drawStr(50, 60, strBuf);

            for (j = 0; j < len; j++)
                strBuf[j] = pgm_read_byte_near(strCmd + j);
            strBuf[j] = 0;

            u8g2.drawStr(50, 35, strBuf);
#ifdef ESP8266
            ESP8266Tasks();
#endif
        } while (u8g2.nextPage());

        lgPM(strSendingIR);
        lgPM(strColonSeparator);
        lgPM(strCmd);
        lgPM(str3DotSeparator);

        sendIR(cmd, cmdLen, configPinLED, configLEDCarrierFreq, cmdDivider);

        lgPM(strDone);
        lgPM(str3DotSeparator);

        beep(false);

        while (!isElapsedTimeFromTS(lastIRSentMillis, configIRCmdDelayMillis)) {
            ignoreIRResults();
#ifdef ESP8266
            ESP8266Tasks();
#endif
        }

        log();
    }

    on = true;
    off = false;
    screenUpdateFlag = true;
    sendingCmdsFlag = false;
}


void printAndSendCmdsON() {
    printAndSendCmds(strON, commandOn, theConfig.commandOnLen, theConfig.commandOnDivider, tempMaxActiveFlag,
                     tempMinActiveFlag, configIRCmdRepeats);
}


void printAndSendCmdsOFF() {
    printAndSendCmds(strOFF, commandOff, theConfig.commandOffLen, theConfig.commandOffDivider, tempMinActiveFlag,
                     tempMaxActiveFlag, configIRCmdRepeats);
}


void alarmHandler() {
    if (curTemp >= tempDeltaMaxValue) {
        alarmFlag = true;

        if (isElapsedTimeFromTS(lastBuzzerAlarmMillis, configBuzzerAlarmMillis[curBuzzerAlarm])) {
            if (configBuzzerAlarmHz[curBuzzerAlarm] > 0)
                tone(configPinBeeper, configBuzzerAlarmHz[curBuzzerAlarm],
                     configBuzzerAlarmMillis[curBuzzerAlarm]);

            if (curBuzzerAlarm == buzzerAlarmMelodyLen - 1)
                curBuzzerAlarm = 0;
            else
                curBuzzerAlarm++;

            lastBuzzerAlarmMillis = millis();
            alarmBuzzerFlag = true;
        }
    } else
        alarmFlag = false;
}


double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#ifdef ESP8266

void ESP8266Tasks() {
    yield();
    server.handleClient();

    if ((WiFi.status() == WL_CONNECTED) && (WLANConnectedFlag == false)) {
        WLANConnectedFlag = true;

        lgPM(strWiFiStatusChangedTo);
        lgPM(strColonSeparator);
        logPM(strON);

        lgPM(strConnectedTo);
        lgPM(strColonSeparator);
        lg(WiFi.SSID());
        lgPM(strDivideSeparator);
        log(WiFi.localIP());
    } else if ((WiFi.status() != WL_CONNECTED) && (WLANConnectedFlag == true)) {
        WLANConnectedFlag = false;

        lgPM(strWiFiStatusChangedTo);
        lgPM(strColonSeparator);
        logPM(strOFF);
    }

    if (beepFlag && isElapsedTimeFromTS(lastBeepMillis, configBuzzerBeepMillis)) {
        beepFlag = false;
        noTone(configPinBeeper);
    }

    if (alarmBuzzerFlag && isElapsedTimeFromTS(lastBuzzerAlarmMillis, configBuzzerAlarmMillis[curBuzzerAlarm])) {
        alarmBuzzerFlag = false;
        noTone(configPinBeeper);
    }
}


void APConfigCallback(WiFiManager *myWiFiManager) {
    char tmpBuf[32], tmpBuf2[32], tmpBuf3[32];

    beep();
    logPM(strAPConfig);
    log(myWiFiManager->getConfigPortalSSID());

    sprintf(tmpBuf2, "%i", ESP.getChipId());

    log(tmpBuf2);
    log(strBuf2);
    log(WiFi.softAPIP());

    strcpy_P(tmpBuf3, strTO);
    strcat_P(tmpBuf3, strColonSeparator);
    sprintf(tmpBuf3, "%s%i", tmpBuf3, configOnConfigAPTimeOutSeconds);
    log(tmpBuf3);

    u8g2.firstPage();
    setFont1;
    do {
        strcpy_P(tmpBuf, strAPConfig);
        u8g2.drawStr(0, 12, tmpBuf);
        u8g2.drawStr(0, 30, tmpBuf2);
        u8g2.drawStr(0, 47, strBuf2);
        u8g2.drawStr(0, 64, tmpBuf3);

        ESP8266Tasks();
    } while (u8g2.nextPage());
}


void handleURIRoot() {
    char buf[512];
    StaticJsonBuffer<512> jsonBuffer;
    JsonObject &json = jsonBuffer.createObject();

    // Calculate uptime
    unsigned long curMillis = millis();
    unsigned long sec = curMillis / 1000;
    unsigned int millis = curMillis - sec * 1000;

    if (prevMillis > curMillis) {
        addSec += 4294967;
        addMillis += 296;
    }

    millis += addMillis;

    if (millis >= 1000) {
        sec++;
        millis -= 1000;
    }

    sec += addSec;
    prevMillis = curMillis;

    sprintf(strBuf, "%lu.%i", sec, millis);
    json["uptime"] = strBuf;
    // End

    json["millis"] = curMillis;
    json["temp"] = curTemp;
    json["tempMin"] = theConfig.tempMin;
    json["tempMax"] = theConfig.tempMax;
    json["tempMaxAlarm"] = tempDeltaMaxValue;
    json["tempGrowStatus"] = byte(tempGrowFlag);
    json["alarmStatus"] = byte(alarmFlag);
    json["status"] = status;
    json["name"] = configOnConfigAPName;
    json["id"] = ESP.getChipId();
    json["WLANStatus"] = byte(WLANConnectedFlag);
    json["freeHeap"] = ESP.getFreeHeap();

    if (server.arg("pretty") == "1")
        json.prettyPrintTo(buf, sizeof(buf));
    else
        json.printTo(buf, sizeof(buf));

    strcpy_P(strBuf, strTextPlain);
    server.send(200, strBuf, buf);
}


void handleURIOn() {
    strcpy_P(strBuf, strTextPlain);

    if (sendingCmdsFlag) {
        strcpy_P(strBuf2, strBusy);
        server.send(200, strBuf, strBuf2);
    } else {
        strcpy_P(strBuf2, strON);
        server.send(200, strBuf, strBuf2);

        printAndSendCmdsON();
    }
}


void handleURIOff() {
    strcpy_P(strBuf, strTextPlain);

    if (sendingCmdsFlag) {
        strcpy_P(strBuf2, strBusy);
        server.send(200, strBuf, strBuf2);
    } else {
        strcpy_P(strBuf2, strOFF);
        server.send(200, strBuf, strBuf2);

        printAndSendCmdsOFF();
    }
}


void handleURISetup() {
    strcpy_P(strBuf, strTextPlain);
    strcpy_P(strBuf2, strSetupMode);
    server.send(200, strBuf, strBuf2);

    setupModeFlag = true;
    lastSetupModeButtonPressMillis = millis();
    i1 = 0;
    b1 = false;
    beep();
}


void handleURIReset() {
    strcpy_P(strBuf, strTextPlain);
    server.send(200, strBuf, server.uri());

    ESP.reset();
}


void handleURITest() {
    strcpy_P(strBuf, strTextPlain);
    server.send(200, strBuf, server.uri());
}


void startHTTP() {
    lgPM(strStartingHTTPServer);
    lgPM(str3DotSeparator);

    server.on("/", handleURIRoot);
    server.on("/on", handleURIOn);
    server.on("/off", handleURIOff);
    server.on("/setup", handleURISetup);
    server.on("/reset", handleURIReset);
    server.on("/test", handleURITest);

    server.begin();

    logPM(strDone);
}


void setupWLAN() {
    WiFiManager wifiManager;

    // Reset settings, only for testing! This will reset your WiFi password
    //wifiManager.resetSettings();

    wifiManager.setTimeout(configOnConfigAPTimeOutSeconds);
    wifiManager.setAPCallback(APConfigCallback);

    sprintf(strBuf, "%s-%i-%i", configOnConfigAPName, ESP.getChipId(), micros());
    randomSeed(micros());
    sprintf(strBuf2, "%08i", rand());

    if (!wifiManager.autoConnect(strBuf, strBuf2))
        logPM(strTimedOut);

    startHTTP();
}

#endif
// End Functions


void setup() {
    Serial.begin(configSerialSpeed);

    while (!Serial) {
#ifdef ESP8266
        ESP8266Tasks();
#endif
    }

    logPM(strStart);
    beep();

    u8g2.begin();
    u8g2.firstPage();
    setFont1;
    do {
        strcpy_P(strBuf, strInitialize);
        u8g2.drawStr(0, 36, strBuf);
#ifdef ESP8266
        ESP8266Tasks();
#endif
    } while (u8g2.nextPage());

    pinMode(configPinTSOP, INPUT);
    pinMode(configPinLED, OUTPUT);
    pinMode(configPinBeeper, OUTPUT);

    findTempSensor();
    calculateEEPROMStartAddresses();
    loadConfig();
    correctConfigValues();

    lgPM(strEEPROMDeviceAddress);
    lgPM(strColonSeparator);
    printDeviceAddress(theConfig.tempSensorDeviceAddress);

    lgPM(strEEPROMSensorResolution);
    lgPM(strColonSeparator);
    log(theConfig.tempSensorResolution);

    // Is real tempDeviceAddress and address in config different?
    b = false;
    for (i = 0; i < sizeof(tempDeviceAddress) / sizeof(tempDeviceAddress[0]); i++) {
        if (theConfig.tempSensorDeviceAddress[i] != tempDeviceAddress[i]) {
            b = true;
            break;
        }
    }
    //End

    // Write tempSensorDeviceAddress / Resolution in EEPROM if it necessary
    if (b || (configTempSensorResolution != theConfig.tempSensorResolution)) {
        logPM(strEEPROMDifferentData);

        theConfig.tempSensorResolution = configTempSensorResolution;

        for (i = 0; i < sizeof(tempDeviceAddress) / sizeof(tempDeviceAddress[0]); i++)
            theConfig.tempSensorDeviceAddress[i] = tempDeviceAddress[i];

        tempSensor.setResolution(tempDeviceAddress, configTempSensorResolution);
        updateConfig(1);
    }
    // End

    requestTemp();
    tempSensor.setWaitForConversion(false);
    tempSensorWaitDataDelayMillis = 750 / (1 << (12 - configTempSensorResolution));
    buzzerAlarmMelodyLen = sizeof(configBuzzerAlarmHz) / sizeof(configBuzzerAlarmHz[0]);
    b1 = false;
    i = 0;
    i1 = 0;

#ifdef ESP8266
    u8g2.firstPage();
    setFont1;
    do {
        strcpy_P(strBuf, strWaiting);
        u8g2.drawStr(0, 12, strBuf);

        strcpy_P(strBuf, strWiFi);
        u8g2.drawStr(0, 36, strBuf);

        ESP8266Tasks();
    } while (u8g2.nextPage());
#endif
}


void loop() {
    if (setupModeFlag) {
        // Timeout SetupMode
        if (isElapsedTimeFromTS(lastSetupModeButtonPressMillis, configMaxWaitSetupModeMillis)) {
            lgPM(strSetupMode);
            lgPM(strColonSeparator);
            logPM(strOFF);

            IRRecv.resume();
            beep();

            setupModeFlag = false;
            screenUpdateFlag = true;
        }
        // End

        if (b1) {
            // Record IR
            if (i1 <= 5) {
                // Record common IR
                if (IRRecv.decode(&IRResults)) {

                    lgFmt(IRResults.value, HEX);
                    lgPM(strCommaSeparator);
                    lg(IRResults.decode_type);
                    lgPM(strColonSeparator);

                    if (IRResults.decode_type == -1) {
                        logPM(strIgnoredIR);
                        IRRecv.resume();
                    } else {
                        switch (i1) {
                            case 0:
                                theConfig.buttonMinUp = IRResults.value;
                                break;
                            case 1:
                                theConfig.buttonMinDown = IRResults.value;
                                break;
                            case 2:
                                theConfig.buttonMaxUp = IRResults.value;
                                break;
                            case 3:
                                theConfig.buttonMaxDown = IRResults.value;
                                break;
                            case 4:
                                theConfig.buttonDeltaMaxUp = IRResults.value;
                                break;
                            case 5:
                                theConfig.buttonDeltaMaxDown = IRResults.value;
                                break;
                            default:
                                break;
                        }

                        strcpy_P(strBuf, (char *) pgm_read_dword(&(strSetupButtons[i1])));
                        log(strBuf);

                        beep();

                        if (i1 == 5)
                            updateConfig(1);
                        else
                            IRRecv.resume();

                        screenDelayFlag = true;
                        lastSetupModeButtonPressMillis = millis();
                        b1 = false;
                        i1++;
                    }
                }
                // End Record common IR
            } else {
                // Record Air Cond IR
                switch (i1) {
                    case 6:
                        do {
                            captureRawIRData(commandOn, commandOff, commandOn, theConfig.commandOnLen,
                                             theConfig.commandOnDivider);
                        } while (theConfig.commandOnLen <= 10);

                        updateConfig(2);
                        break;
                    case 7:
                        do {
                            captureRawIRData(commandOn, commandOff, commandOff, theConfig.commandOffLen,
                                             theConfig.commandOffDivider);
                        } while (theConfig.commandOffLen <= 10);

                        updateConfig(3);
                        break;
                    default:
                        break;
                }

                beep();

                if (i1 == 7) {
                    beep();
                    updateConfig(1);
                    loadConfig();

                    lgPM(strSetupMode);
                    lgPM(strColonSeparator);
                    lgPM(strOFF);
                    ignoreIRResults();
                    log();
                    setupModeFlag = false;
                    screenUpdateFlag = true;
                } else {
                    lastSetupModeButtonPressMillis = millis();
                    b1 = false;
                    i1++;
                }
                // End
            }
            // End Record IR
        } else {
            // Print SetupMode dialog, i1 - setup scenario id, b1 - screen updated flag
            lgPM(strSetupMode);
            lgPM(strColonSeparator);
            lg(i1);
            lgPM(strColonSeparator);
            lgPM(strPrintingScreen);
            lgPM(str3DotSeparator);

            u8g2.firstPage();
            setFont1;
            do {
                strcpy_P(strBuf, strPressButton);
                u8g2.drawStr(0, 60, strBuf);

                strcpy_P(strBuf, strSetupMode);
                u8g2.drawStr(0, 11, strBuf);

                strcpy_P(strBuf, (char *) pgm_read_dword(&(strSetupButtons[i1])));
                u8g2.drawStr(0, 35, strBuf);
#ifdef ESP8266
                ESP8266Tasks();
#endif
            } while (u8g2.nextPage());

            lgPM(strDone);

            if (screenDelayFlag) {
                while (!isElapsedTimeFromTS(lastSetupModeButtonPressMillis, configScreenDelayOnKeyPressMillis)) {
                    ignoreIRResults();
#ifdef ESP8266
                    ESP8266Tasks();
#endif
                }

                screenDelayFlag = false;
            }

            log();
            b1 = true;
            // End Print SetupMode dialog
        }
    } else {
        if (gotTempFlag) {
            // Print main screen
            if (screenUpdateFlag) {
                lgPM(strPrintingScreen);
                lgPM(str3DotSeparator);

                tempDeltaMaxValue = theConfig.tempMax + theConfig.tempDeltaMax;

                u8g2.firstPage();
                do {
                    setFont1;

                    strcpy_P(strBuf2, strON);
                    sprintf(strBuf, "%i %s", theConfig.tempMax, strBuf2);
                    u8g2.drawStr(0, 13, strBuf);

                    sprintf(strBuf, "%i(%i)", tempDeltaMaxValue, theConfig.tempDeltaMax);
                    u8g2.drawStr(75, 13, strBuf);

                    strcpy_P(strBuf2, strOFF);
                    sprintf(strBuf, "%i %s", theConfig.tempMin, strBuf2);
                    u8g2.drawStr(0, 63, strBuf);

                    if (alarmFlag) {
                        strcpy(strBuf, "!!!");
                    } else {
                        if ((tempMinActiveFlag) || (tempMaxActiveFlag)) {
                            if (tempMinActiveFlag) {
                                strcpy_P(strBuf, strOFF);
                                status = 0;
                            } else {
                                strcpy_P(strBuf, strON);
                                status = 1;
                            }
                        } else {
                            strcpy(strBuf, "???");
                            status = 2;
                        }
                    }
                    u8g2.drawStr(95, 63, strBuf);

                    setFont2;

                    dtostrf(curTemp, 2, 2, strBuf);
                    sprintf(strBuf2, "%6s", strBuf);
                    u8g2.drawStr(35, 42, strBuf2);

                    // Draw position scale + arrows
                    i1 = 20;
                    i2 = 43;

                    for (i = i1; i < i2; i += 2)
                        u8g2.drawPixel(12, i);

                    // Draw horizontal arrow
                    tmpTemp = curTemp;

                    if (curTemp < theConfig.tempMin)
                        tmpTemp = theConfig.tempMin;

                    if (curTemp > theConfig.tempMax)
                        tmpTemp = theConfig.tempMax;

                    i = i2 - (mapDouble(tmpTemp, theConfig.tempMin, theConfig.tempMax, i1, i2) - i1);

                    i1 = 16;

                    u8g2.drawLine(i1, i, i1 + 17, i);

                    for (i2 = 1; i2 <= 4; i2++) {
                        t1 = i1 + i2 * 2;
                        t2 = t1 + 1;

                        u8g2.drawLine(t1, i - i2, t2, i - i2);
                        u8g2.drawLine(t1, i + i2, t2, i + i2);
                    }
                    // End

                    // Draw vertical arrow
                    i1 = 4;
                    i = 22;

                    u8g2.drawLine(i1, i, i1, i + 17);

                    if (!tempGrowFlag)
                        i += 17;

                    for (i2 = 1; i2 <= 4; i2++) {

                        if (tempGrowFlag) {
                            t1 = i + i2 * 2;
                            t2 = t1 + 1;
                        } else {
                            t1 = i - i2 * 2;
                            t2 = t1 - 1;
                        }

                        u8g2.drawLine(i1 + i2, t1, i1 + i2, t2);
                        u8g2.drawLine(i1 - i2, t1, i1 - i2, t2);
                    }
                    // End
                    // End Draw position scale + arrows
#ifdef ESP8266
                    ESP8266Tasks();
#endif
                    alarmHandler();
                } while (u8g2.nextPage());

                lgPM(strDone);
                screenUpdateFlag = false;

                if (screenDelayFlag) {
                    while (!isElapsedTimeFromTS(lastScreenDelayMillis, configScreenDelayOnKeyPressMillis)) {
                        ignoreIRResults();
#ifdef ESP8266
                        ESP8266Tasks();
#endif
                    }

                    screenDelayFlag = false;
                }

                log();
            }
            // End Print main screen

            // Receive IR command
            if (IRRecv.decode(&IRResults)) {
                lgPM(strGotIR);
                lgPM(strColonSeparator);
                lgFmt(IRResults.value, HEX);
                lgPM(strCommaSeparator);
                log(IRResults.decode_type);

                // IR command handler
                if (IRResults.value == theConfig.buttonMinUp) {
                    logPM(strSetupButton_0);
                    theConfig.tempMin++;
                    screenUpdateFlag = true;
                } else if (IRResults.value == theConfig.buttonMinDown) {
                    logPM(strSetupButton_1);
                    theConfig.tempMin--;
                    screenUpdateFlag = true;
                } else if (IRResults.value == theConfig.buttonMaxUp) {
                    logPM(strSetupButton_2);
                    theConfig.tempMax++;
                    screenUpdateFlag = true;
                } else if (IRResults.value == theConfig.buttonMaxDown) {
                    logPM(strSetupButton_3);
                    theConfig.tempMax--;
                    screenUpdateFlag = true;
                } else if (IRResults.value == theConfig.buttonDeltaMaxUp) {
                    logPM(strSetupButton_4);
                    theConfig.tempDeltaMax++;
                    screenUpdateFlag = true;
                } else if (IRResults.value == theConfig.buttonDeltaMaxDown) {
                    logPM(strSetupButton_5);
                    theConfig.tempDeltaMax--;
                    screenUpdateFlag = true;
                }

                correctConfigValues();

                if (screenUpdateFlag) {
                    screenDelayFlag = true;
                    lastScreenDelayMillis = millis();

                    beep();
                    updateConfig(1);
                }
                // End IR handler command

                IRRecv.resume();
            }
            // End Receive IR command

            // Temperature handler
            if ((curTemp >= theConfig.tempMax) && (!tempMaxActiveFlag))
                printAndSendCmdsON();

            if ((curTemp <= theConfig.tempMin) && (!tempMinActiveFlag))
                printAndSendCmdsOFF();

            alarmHandler();
            // End Temperature handler
        } else {
            // While gotTempFlag false (temperature sensor doesn't ready to return a data) trying to catch setupMode attempt
            if (b1)
                tone(configPinLED, configLEDCarrierFreq);

            delayMicroseconds(500);

            b = digitalRead(configPinTSOP);

            if (b1)
                noTone(configPinLED);

            if ((b && !b1) || (!b && b1))
                i1++;

            b1 = !b1;
            i++;
            // End
        }

        // Read temperature sensor if it ready
        if (isElapsedTimeFromTS(lastTempRequestMillis, tempSensorWaitDataDelayMillis)) {
            readTempSensorData();
            requestTemp();
            ignoreIRResults();
            log();
        }
        // End
    }

#ifdef ESP8266
    ESP8266Tasks();
#endif
}
