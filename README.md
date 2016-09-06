# AirCondIRManager (Air Conditioner IR Manager)
This is an Arduino project to control your air conditioner automatically.

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
