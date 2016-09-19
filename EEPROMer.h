#include <Arduino.h>
#include <EEPROM.h>

template<class T>
unsigned int EEPROMReader(unsigned int startAddress, T &value, unsigned int totalSize = 0) {
    byte *p = (byte *) (void *) &value;
    unsigned int i, size = sizeof(value);

#ifdef ESP8266
    EEPROM.begin(totalSize);
#endif

    for (i = 0; i < size; i++)
        *p++ = EEPROM.read(startAddress++);

    return i;
}

template<class T>
unsigned int EEPROMUpdater(unsigned int startAddress, const T &value, unsigned int totalSize = 0) {
    const byte *byteValue = (const byte *) (const void *) &value;
    byte b;
    unsigned int i, size = sizeof(value);

#ifdef ESP8266
    EEPROM.begin(totalSize);
#endif

    for (i = 0; i < size; i++) {
        b = EEPROM.read(startAddress);

        if (b != *byteValue)
            EEPROM.write(startAddress, *byteValue);

        startAddress++;
        *byteValue++;
    }

#ifdef ESP8266
    EEPROM.commit();
    EEPROM.end();
#endif

    return i;
}
