# AirCondIRManager (Air Conditioner IR Manager)
This is an Arduino/ESP8266 project to control your air conditioner automatically.

# Preface
I made some kind of replacement of a industrial chiller for water cooling:
http://www.teyuchiller.com/Products/ChillerCW5000800Wcoo.html  
I used Electrolux EACM-DR/N3 portable air conditioner: 
while it running, it cooling at a vehicle radiator (VAZ-2108 in my case).  
Water pomp continuously pumping water and as a result, water temperature decreasing...

# Problem
My system works pretty well, but I wanted to control when an air conditioner
turns on and off, because while it running, it consumes a lot of electricity.

# Goal
The goal is management when an air conditioner turns on and off based on water temperature.
Also I wanted to create an easy configurable system (easy to change temperature thresholds).

# Components
In this project I used:
 - Arduino (tested with Arduino Uno)
 - IR LED for emulating an air conditioner IR remote control
 - IR receiver (I used TSOP38238)
 - Waterproof DS18B20 temperature sensor
 - Speaker
 - SSD1306 OLED display

# Results Description
Built system has a configuration mode. To go there you need to place IR LED
closer the IR receiver and turn power on.

At the setup mode you able to setup 6 buttons
(use any remote control, except an air conditioner remote control):
- temperature minimum up
- temperature minimum down
- temperature maximum up
- temperature maximum down
- temperature delta maximum up
- temperature delta maximum down

and two buttons related to an air conditioner:
- turn air conditioner on
- turn air conditioner off

System outputs main parameters at the OLED screen:
- current temperature
- temperature minimum
- temperature maximum
- temperature delta maximum  
You can change last three parameters via configured remote control in runtime.

Wifi features (ESP8266 only):
 - WiFi access for easy configuring:
   - creates WiFi AP to connect and provide real WiFi credentials (password randomly generated and printed at OLED screen)
 - returns current status in json (http://esp8266-ip-address/, http://esp8266-ip-address/?pretty=1)
   - temperatures: current, minimum, maximum, alarm maximum, direction (grow or not)
   - air conditioner status (0 - off, 1 - on, 2 - unknown)
   - alarm flag
   - uptime in microseconds
   - free heap memory
   - uptime
   - example: {"uptime":"13152.50","millis":13152050,"temp":20.25,"tempMin":18,"tempMax":24,"tempAlarm":25,"status":2,"tempGrowStatus":0,"alarmStatus":0,"name":"AirCondManager","id":1067883,"WLANStatus":1,"freeHeap":35712}
 - Turns on, turns off air conditioner by sending commands:
   - for turning on:  http://esp8266-ip-address/on
   - for turning off: http://esp8266-ip-address/off
 - Set temperature settings (because of this, there is a way to use the device without TSOP and IR remote control):
   - temperature minimum: http://esp8266-ip-address/set/?tempMin=19
   - temperature maximum: http://esp8266-ip-address/set/?tempMax=24
   - temperature alarm:   http://esp8266-ip-address/set/?tempAlarm=25
 - Goes to Setup Mode: http://esp8266-ip-address/setup
 - Reboots: http://esp8266-ip-address/reset
 - Dummy test response: http://esp8266-ip-address/test

# Work Logic
If current temperature more then "temperature maximum",  
then the system sends IR recorded command to air conditioner to turn it on.

If current temperature less then "temperature minimum",  
then the system sends IR recorded command to air conditioner to turn it off.

If current temperature more then "temperature maximum" plus "temperature delta maximum", 
then an alarm sound activated.
---


Good Luck and Best Regards,  
Alexey Tsarev, Tsarev.Alexey at gmail.com
