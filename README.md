# sump_mon

ardrino ide project to use ESP32-CAM as sump pump monitor.
YouTube Link:  https://youtu.be/eNzcDllqPsk

## To compile:

modify dummy_config.h file to match your home wifi, time zone settings and Telegram Bot token. Save the file as "my_config.h" before compile.

## Features:

+ self-learning
+ catch common motor failure modes.
+ send Telegram alarm message when abnormal conditions appear
+ built-in web interface with real-time camera view
+ web page shows time and duration of last 30 pump events

## Hardware needed

+ current sensor (5A/5mA CT)
+ bridge rectifier diods
+ 3.3V zener diode
