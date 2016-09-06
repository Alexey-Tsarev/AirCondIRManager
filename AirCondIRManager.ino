#include <DallasTemperature.h>
#include <EEPROMex.h>
#include <U8glib.h>

// IRremoteInt.h - Change to use #define IR_USE_TIMER1
//                 Change to minimal for your IR transmitter value at the #define RAWBUF
//                 My case - #define RAWBUF 70
// IRremote.h - Turn off all #define SEND_*, #define DECODE_*
//              Turn on only yours DECODE_*
//              My case - turned on DECODE_RC5, DECODE_SAMSUNG
#include <IRremote.h>


// Cfg
#define configPinLED 3
#define configPinTSOP 4
#define configPinTempSensor 5
#define configPinBeeper 6
#define configTempSensorResolution 12 // 9 - 12
#define configMaxWaitSetupModeMillis 10000
#define configScreenDelayOnKeyPressMillis 250
#define configLEDCarrierFreq 38000
#define configSerialSpeed 115200
#define configEEPROMMaxAllowedWrites 32767
#define configIRCmdRepeats 5
#define configIRCmdDelayMillis 1000
#define configDisplayTempMin 10
#define configDisplayTempMax 40
#define configDisplayDeltaTempMin 1
#define configDisplayDeltaTempMax 9
unsigned int configBuzzerBeepHz = 2000;
unsigned int configBuzzerBeepMillis = 150;
unsigned int configBuzzerAlarmHz[] = {1000, 1500, 1200, 1300};
unsigned int configBuzzerAlarmMillis[] = {200, 200, 200, 200};
// End Cfg

//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST); // Fast I2C / TWI
U8GLIB_SSD1306_128X64 u8g(11, 10, U8G_PIN_NONE, 8, 9);

#define capDataMaxLen 232 // My air cond has 230, Electrolux has 68
#define maxWaitCapDataMicros 1000000 // No IR signal time, 1 sec = 10^6 micros

struct config {
    byte tempSensorDeviceAddress[8];
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

byte tempDeltaMaxValue, buzzerAlarmMelodyLen, curBuzzerAlarm;
bool b, b1, gotTempFlag, setupModeFlag, screenUpdateFlag, screenDelayFlag, tempMinActive, tempMaxActive;
char strBuf[16];
unsigned int i, i1, tempSensorDelayInMillis, startAddressConfig, startAddressConfigCommandOn, startAddressConfigCommandOff;
unsigned long lastTempRequestMillis, lastSetupModeButtonPressMillis, lastScreenDelay, lastBuzzerAlarm;
float curTemp = 0, prevTemp = -274;

byte tempDeviceAddress[8];
OneWire oneWire(configPinTempSensor);
DallasTemperature tempSensor(&oneWire);
IRrecv irrecv(configPinTSOP);
decode_results IRResults;

#define lg(str)          Serial.print(str)
#define log(str)         Serial.println(str)
#define lgFmt(str, fmt)  Serial.print(str,fmt)

// Strings at PROGMEM
const char strStart[] PROGMEM = "Start";
const char strReadingEEPROM[] PROGMEM = "Reading EEPROM... ";
const char strUpdatingEEPROM[] PROGMEM = "Updating EEPROM... ";
const char strArrayLen[] PROGMEM = "Array len";
const char strCapturedRaw[] PROGMEM = "Captured raw";
const char strFindingMax[] PROGMEM = "Finding max";
const char strDividedBy[] PROGMEM = "Divided by";
const char strMultipliedBy[] PROGMEM = "Multiplied by";
const char strFoundTempSensor[] PROGMEM = "Found temperature sensor(s)";
const char strTempSensorNotFound[] PROGMEM = "A temperature sensor not found";
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
const char strGettingTemp[] PROGMEM = "Getting temp... ";
const char strGotIR[] PROGMEM = "Got IR";
const char strPrintingScreen[] PROGMEM = "Printing screen... ";
const char strIgnoredIR[] PROGMEM = "Ignored IR";
const char strBeepStarts[] PROGMEM = "Beep";
const char strSendingIR[] PROGMEM = "Sending IR";
const char strStartAddress[] PROGMEM = "Start address";
const char strCommaSeparator[] PROGMEM = ", ";
const char strDotSeparator[] PROGMEM = ". ";
const char strDivideSeparator[] PROGMEM = " / ";
const char strColonSeparator[] PROGMEM = ": ";
const char strSetupMode[] PROGMEM = "SetupMode";
const char strPressButton[] PROGMEM = "PressButton";
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
// End Strings

// Functions
void lgPM(char *s) {
    unsigned int i, len = strlen_P(s);
    char str;

    for (i = 0; i < len; i++) {
        str = (char) pgm_read_byte_near(s + i);
        lg(str);
    }
}


void logPM(char *s) {
    lgPM(s);
    log();
}


void delayMicrosecondsAccurate(unsigned long waitMicros) {
    if (waitMicros > 4) {
        unsigned long start = micros();
        unsigned long endMicros = start + waitMicros - 4;

        if (endMicros < start) // Wait if overflow
            while (micros() > start);

        while (micros() < endMicros); // Normal wait
    }
}


void ir_send(byte raw[], byte rawLen, byte pinLED, unsigned int carrierFreq, byte multiplier = 1) {
    byte startPos, i;
    bool curBit = true;

    if (raw[0] == 0)
        startPos = 1;
    else
        startPos = 0;

    for (i = startPos; i < rawLen; i++) {
        if (curBit)
            tone(pinLED, carrierFreq);

        if (multiplier == 1)
            delayMicrosecondsAccurate(raw[i]);
        else
            delayMicrosecondsAccurate(raw[i] * multiplier);

        if (curBit)
            noTone(pinLED);

        curBit = !curBit;
    }
}


void printDeviceAddress(DeviceAddress deviceAddress) {
    for (byte i = 0; i < 8; i++) {
        if (deviceAddress[i] < 16)
            lg("0");

        lgFmt(deviceAddress[i], HEX);
    }
}


bool isElapsedTimeFromStart(unsigned long start, unsigned long elapsed, bool is_micros = false) {
    unsigned long cur;
    unsigned long delta;

    if (is_micros)
        cur = micros();
    else
        cur = millis();

    if (cur >= start)
        delta = cur - start;
    else
        delta = 0xFFFFFFFF - start + cur + 1;

    return delta > elapsed;
}

void captureRawIRData(byte cap1[], byte cap2[], byte output[], byte &outputLen, byte &outputDivider) {
    bool startBit, waitBit, curBit, maxWaitMillisFlag;
    unsigned int i, j, max;
    unsigned long lastTS, curMicros;

    startBit = digitalRead(configPinTSOP);
    waitBit = !startBit;

    maxWaitMillisFlag = false;
    i = 0;
    lastTS = 0;

    while (i < capDataMaxLen) {
        do {
            curBit = digitalRead(configPinTSOP);

            if ((i >= 1) && isElapsedTimeFromStart(lastTS, maxWaitCapDataMicros, true)) {
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
    }
    log();

    lgPM(strFindingMax);
    lgPM(strColonSeparator);
    max = word(cap1[1], cap2[1]);
    for (j = 2; j < i; j++)
        if (word(cap1[j], cap2[j]) > max)
            max = word(cap1[j], cap2[j]);
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
            logPM(strTempSensorNotFound);
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
        log();

        break;
    }
}


void calculateEEPROMStartAddresses() {
    unsigned int startAddress = 0;
    startAddressConfig = startAddress;

    lgPM(strStartAddress);
    lgPM(strColonSeparator);
    log(startAddressConfig);

    startAddress += sizeof(theConfig);
    startAddressConfigCommandOn = startAddress;

    lgPM(strStartAddress);
    lgPM(strColonSeparator);
    log(startAddressConfigCommandOn);

    startAddress += sizeof(commandOn);
    startAddressConfigCommandOff = startAddress;

    lgPM(strStartAddress);
    lgPM(strColonSeparator);
    log(startAddressConfigCommandOff);
}


void loadConfigs() {
    EEPROM.setMaxAllowedWrites(configEEPROMMaxAllowedWrites);

    lgPM(strReadingEEPROM);
    EEPROM.readBlock(startAddressConfig, theConfig);
    EEPROM.readBlock(startAddressConfigCommandOn, commandOn);
    EEPROM.readBlock(startAddressConfigCommandOff, commandOff);
    logPM(strDone);
}


void updateConfigs(byte updateConfigBlock = 0) {
    lgPM(strUpdatingEEPROM);

    if ((updateConfigBlock == 0) || (updateConfigBlock == 1))
        EEPROM.updateBlock(startAddressConfig, theConfig);

    if ((updateConfigBlock == 0) || (updateConfigBlock == 2))
        EEPROM.updateBlock(startAddressConfigCommandOn, commandOn);

    if ((updateConfigBlock == 0) || (updateConfigBlock == 3))
        EEPROM.updateBlock(startAddressConfigCommandOff, commandOff);

    logPM(strDone);
}


void beep(bool newLine = true) {
    lgPM(strBeepStarts);

    tone(configPinBeeper, configBuzzerBeepHz, configBuzzerBeepMillis);

    if (newLine)
        log();
}


void readTempSensorData() {
    lgPM(strGettingTemp);

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
        lgPM(strSetupMode);
        lgPM(strColonSeparator);
        lg(i1);
        lgPM(strDivideSeparator);
        lg(i);
        lgPM(strColonSeparator);

        noTone(configPinLED);

        irrecv.enableIRIn();
        irrecv.blink13(true);

        if (i == i1) {
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
        prevTemp = curTemp;
        screenUpdateFlag = true;
    }

    requestTemp();
}


void correctTempValues() {
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
}


void ignoreIRResults() {
    if (irrecv.decode(&IRResults)) {
        lgPM(strDotSeparator);
        lgPM(strIgnoredIR);
        lgPM(strColonSeparator);
        lgFmt(IRResults.value, HEX);
        lgPM(strCommaSeparator);
        lg(IRResults.decode_type);

        irrecv.resume();
    }
}


void printAndSendCmds(char strCmd[], byte cmd[capDataMaxLen], byte cmdLen, byte cmdDivider, bool &on, bool &off) {
    byte i, len = strlen_P(strCmd);

    u8g.firstPage();
    do {
        dtostrf(curTemp, 2, 2, strBuf);
        u8g.drawStr(40, 15, strBuf);

        for (i = 0; i < len; i++)
            strBuf[i] = pgm_read_byte_near(strCmd + i);
        strBuf[i] = 0;

        u8g.setScale2x2();
        u8g.drawStr(20, 30, strBuf);
        u8g.undoScale();
    } while (u8g.nextPage());

    for (i = 0; i < configIRCmdRepeats; i++) {
        lgPM(strSendingIR);
        lgPM(strColonSeparator);
        log(strBuf);

        ir_send(cmd, cmdLen, configPinLED, configLEDCarrierFreq,
                cmdDivider);

        beep();
        delay(configIRCmdDelayMillis);
    }

    on = true;
    off = false;
    screenUpdateFlag = true;
}
// End Functions


void setup() {
    Serial.begin(configSerialSpeed);
    while (!Serial);

    logPM(strStart);

    pinMode(configPinTSOP, INPUT);
    pinMode(configPinLED, OUTPUT);
    pinMode(configPinBeeper, OUTPUT);

    findTempSensor();
    calculateEEPROMStartAddresses();
    loadConfigs();

    lgPM(strEEPROMDeviceAddress);
    lgPM(strColonSeparator);
    printDeviceAddress(theConfig.tempSensorDeviceAddress);
    log();

    lgPM(strEEPROMSensorResolution);
    lgPM(strColonSeparator);
    log(theConfig.tempSensorResolution);

    // Are tempDeviceAddress / config different?
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
        updateConfigs(1);
    }
    // End

    requestTemp();
    tempSensor.setWaitForConversion(false);
    tempSensorDelayInMillis = 750 / (1 << (12 - configTempSensorResolution));

    u8g.setFont(u8g_font_courB14);

    buzzerAlarmMelodyLen = sizeof(configBuzzerAlarmHz) / sizeof(configBuzzerAlarmHz[0]);
    gotTempFlag = false;
    setupModeFlag = false;
    b1 = false;
    i = 0;
    i1 = 0;
}


void loop() {
    if (setupModeFlag) {
        // Timeout SetupMode
        if (isElapsedTimeFromStart(lastSetupModeButtonPressMillis, configMaxWaitSetupModeMillis)) {
            lgPM(strSetupMode);
            lgPM(strColonSeparator);
            logPM(strOFF);

            irrecv.resume();

            setupModeFlag = false;
            screenUpdateFlag = true;
        }
        // End

        if (b1) {
            // Record IR
            if (i1 <= 5) {
                // Record common IR
                if (irrecv.decode(&IRResults)) {

                    lgFmt(IRResults.value, HEX);
                    lgPM(strCommaSeparator);
                    log(IRResults.decode_type);
                    lgPM(strColonSeparator);

                    if (IRResults.decode_type == -1) {
                        logPM(strIgnoredIR);
                        irrecv.resume();
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

                        strcpy_P(strBuf, (char *) pgm_read_word(&(strSetupButtons[i1])));
                        log(strBuf);

                        beep();

                        if (i1 == 5)
                            updateConfigs(1);
                        else
                            irrecv.resume();

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
                        captureRawIRData(commandOn, commandOff, commandOn, theConfig.commandOnLen,
                                         theConfig.commandOnDivider);
                        updateConfigs(2);
                        break;
                    case 7:
                        captureRawIRData(commandOn, commandOff, commandOff, theConfig.commandOffLen,
                                         theConfig.commandOffDivider);
                        updateConfigs(3);
                        break;
                    default:
                        break;
                }

                beep();

                if (i1 == 7) {
                    updateConfigs(1);
                    loadConfigs();

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

            u8g.firstPage();
            do {
                strcpy_P(strBuf, strPressButton);
                u8g.drawStr(0, 11, strBuf);

                strcpy_P(strBuf, (char *) pgm_read_word(&(strSetupButtons[i1])));
                u8g.setScale2x2();
                u8g.drawStr(0, 20, strBuf);
                u8g.undoScale();

                strcpy_P(strBuf, strSetupMode);
                u8g.drawStr(0, 60, strBuf);

            } while (u8g.nextPage());

            lgPM(strDone);

            if (screenDelayFlag) {
                while (!isElapsedTimeFromStart(lastSetupModeButtonPressMillis, configScreenDelayOnKeyPressMillis))
                    ignoreIRResults();

                screenDelayFlag = false;
            }

            log();
            b1 = true;
            // End
        }
    } else {
        if (gotTempFlag) {
            // Print main screen
            if (screenUpdateFlag) {
                lgPM(strPrintingScreen);

                tempDeltaMaxValue = theConfig.tempMax + theConfig.tempDeltaMax;

                u8g.firstPage();
                do {
                    dtostrf(curTemp, 2, 2, strBuf);
                    u8g.setScale2x2();
                    u8g.drawStr(5, 30, strBuf);
                    u8g.undoScale();

                    sprintf(strBuf, "%i", theConfig.tempMin);
                    u8g.drawStr(0, 13, strBuf);

                    sprintf(strBuf, "%i", theConfig.tempMax);
                    u8g.drawStr(40, 13, strBuf);

                    sprintf(strBuf, "%i(%i)", tempDeltaMaxValue, theConfig.tempDeltaMax);
                    u8g.drawStr(75, 13, strBuf);
                } while (u8g.nextPage());

                lgPM(strDone);
                screenUpdateFlag = false;

                if (screenDelayFlag) {
                    while (!isElapsedTimeFromStart(lastScreenDelay, configScreenDelayOnKeyPressMillis))
                        ignoreIRResults();

                    screenDelayFlag = false;
                }

                log();
            }
            // End

            // Receive IR command
            if (irrecv.decode(&IRResults)) {
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

                correctTempValues();

                if (screenUpdateFlag) {
                    screenDelayFlag = true;
                    lastScreenDelay = millis();

                    beep();

                    updateConfigs(1);
                }
                // End IR handler command

                irrecv.resume();
            }
            // End Receive IR command

            // Temperature handler
            if (curTemp >= tempDeltaMaxValue) {
                if (isElapsedTimeFromStart(lastBuzzerAlarm, configBuzzerAlarmMillis[curBuzzerAlarm])) {

                    tone(configPinBeeper, configBuzzerAlarmHz[curBuzzerAlarm],
                         configBuzzerAlarmMillis[curBuzzerAlarm]);

                    if (curBuzzerAlarm == buzzerAlarmMelodyLen - 1)
                        curBuzzerAlarm = 0;
                    else
                        curBuzzerAlarm++;

                    lastBuzzerAlarm = millis();
                }
            }

            if ((curTemp >= theConfig.tempMax) && (!tempMaxActive))
                printAndSendCmds(strON, commandOn, theConfig.commandOnLen, theConfig.commandOnDivider, tempMaxActive,
                                 tempMinActive);

            if ((curTemp <= theConfig.tempMin) && (!tempMinActive))
                printAndSendCmds(strOFF, commandOff, theConfig.commandOffLen, theConfig.commandOffDivider,
                                 tempMinActive, tempMaxActive);
            // End Temperature handler
        } else {
            // While gotTempFlag false (temperature sensor doesn't ready to return a data) trying to catch setupMode attempt
            if (b1)
                tone(configPinLED, configLEDCarrierFreq);
            else
                noTone(configPinLED);

            delayMicrosecondsAccurate(512);

            b = digitalRead(configPinTSOP);

            if ((b && !b1) || (!b && b1))
                i1++;

            b1 = !b1;
            i++;
            // End
        }

        // Read if temp sensor ready
        if (isElapsedTimeFromStart(lastTempRequestMillis, tempSensorDelayInMillis)) {
            readTempSensorData();
            ignoreIRResults();
            log();
        }
        // End
    }
}
