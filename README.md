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
 - ESP8266 (Wemos D1) / Arduino (tested with Arduino Uno)
 - IR LED for emulating an air conditioner IR remote control
 - IR receiver (I used TSOP38238)
 - IR remote control (optional, not necessary if an ESP8266 used)
 - Waterproof temperature sensor DS18B20
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
- max on time (minutes) up
- max on time (minutes) down

and two buttons related to an air conditioner:
- turn air conditioner On
- turn air conditioner Off

System outputs main parameters at the OLED screen:
- current temperature
- temperature minimum
- temperature maximum
- temperature delta
- time (minutes) after last condition
- maxOn time (minutes)
You can change last 4 parameters via configured remote control in runtime.

Wifi features (ESP8266 only):
 - WiFi access for easy configuring:
   - creates WiFi AP to connect and provide real WiFi credentials (password randomly generated and printed at OLED screen)
 - returns current status in json (http://esp8266-ip-address/, http://esp8266-ip-address/?pretty=1)
   - temperatures: current, minimum, maximum, alarm, direction (grow or not)
   - air conditioner status (0 - Off, 1 - On, 2 - Unknown)
   - alarm flag
   - uptime in microseconds
   - runtime after changing last condition
   - free heap memory
   - example: {"name":"AirCondManager","id":1067883,"uptime":"317.873","temp":19.31,"tempMin":15,"tempMax":22,"tempAlarm":28,"maxOnCur":5,"maxOn":20,"status":2,"tempGrowStatus":1,"alarmStatus":0,"WiFiStatus":1,"freeHeap":35976}
 - Turns on, turns off air conditioner by sending commands:
   - for turning on:  http://esp8266-ip-address/on
   - for turning off: http://esp8266-ip-address/off
 - Set temperature settings (because of this, there is a way to use the device without TSOP and IR remote control):
   - temperature minimum: http://esp8266-ip-address/set/?tempMin=19
   - temperature maximum: http://esp8266-ip-address/set/?tempMax=24
   - temperature alarm:   http://esp8266-ip-address/set/?tempAlarm=25
 - Set maxOn time:
   - maxOn: http://esp8266-ip-address/set/?maxOn=30
 - Goes to Setup Mode: http://esp8266-ip-address/setup
 - Reboots: http://esp8266-ip-address/reset
 - Dummy test response: http://esp8266-ip-address/test
 - There is a php script for sending parsed json data to Zabbix

# Work Logic
If current temperature is higher than "temperature maximum",
then the system sends IR recorded command to air conditioner to turn it On.

If current temperature is less than "temperature minimum",
then the system sends IR recorded command to air conditioner to turn it Off.

If current temperature is higher than "temperature maximum" plus "temperature delta maximum",
then an alarm sound activated.

If current status is On and temperature between minimum and maximum and elapsed time from last condition
is more then maxOn (minutes), then the system sends IR recorded command to air conditioner to turn it Off.

If current status is On and temperature higher than "temperature maximum" and elapsed time from last condition
is more then maxOn (minutes), then an alarm sound activated.
---


Good Luck and Best Regards,  
Alexey Tsarev, Tsarev.Alexey at gmail.com
