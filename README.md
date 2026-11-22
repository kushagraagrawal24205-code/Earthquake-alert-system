🌍 IoT-Based Earthquake Monitoring and Alert System

An IoT-based earthquake monitoring and alert system developed using **ESP8266, MPU6050 accelerometer, and SX1278 LoRa wireless communication**. The system detects ground vibrations and transmits monitoring information wirelessly to a remote receiver for real-time alerts.

1. Project Overview:

The project aims to develop a low-cost wireless earthquake vibration monitoring system capable of detecting abnormal ground movement and transmitting alerts to a remote monitoring unit.

The transmitter uses an MPU6050 accelerometer to measure vibration and movement. An ESP8266 processes the sensor information and sends data through an SX1278 LoRa module. A second ESP8266 receives the transmitted data and provides visual and audible alerts.

2. Project Objectives:

a.Detect ground vibrations using an accelerometer.
b.Transmit monitoring data wirelessly using LoRa technology.
c.Display received information on an OLED display.
d.Generate audible alerts using a buzzer.
e.Develop a low-cost and scalable monitoring prototype.

3. Hardware Components:

ESP8266 NodeMCU (2)---Transmitter and receiver controllers 
MPU6050-------Vibration and acceleration measurement 
SX1278 LoRa (2)-----Wireless data transmission 
OLED Display (SSD1306)----Display received information 
Buzzer----Audible alert 
LED-----Visual indication 
Antennas (433 MHz)-----LoRa wireless communication 
Power Supply-----Powering the system 

4. System Architecture:

The system consists of two main units:

A. Transmitter Unit
a.MPU6050 measures acceleration and vibration.
b.ESP8266 processes the sensor readings.
c.SX1278 transmits the monitoring information using 433 MHz LoRa communication.

B. Receiver Unit
a.ESP8266 receives wireless data through the SX1278 module.
b.Received information is displayed on the OLED.
c.Buzzer and LED provide alert indications.

5. Working Principle:

1. The MPU6050 continuously monitors acceleration and vibration.
2. The transmitter ESP8266 processes the sensor readings.
3. Monitoring information is transmitted through the LoRa module.
4. The receiver ESP8266 receives the transmitted packets.
5. The OLED displays the received monitoring information.
6. The buzzer and LED provide alerts according to the received status.

6. Wireless Communication:

a.Communication technology: LoRa
b.Module: SX1278
c.Operating frequency: 433 MHz
d.Controller: ESP8266 NodeMCU

LoRa communication is used to establish wireless connectivity between the transmitter and receiver units.

7. Software and Development Tools:

a.Arduino IDE
b.Embedded C/C++ (Arduino framework)
c.ESP8266 Board Support Package
d.RadioLib Library
e.MPU6050 sensor library
f.Adafruit SSD1306 OLED library

8. Receiver Pin Connections:

LoRa NSS---D8 
LoRa DIO0 --D1 
LoRa RESET --D0 
OLED SDA ---D2 
OLED SCL---D4 
Buzzer---D3 

OLED I2C address: `0x3C`

LoRa operating frequency: `433.0 MHz`

*Refer to the firmware and circuit diagram for the complete transmitter connections.*

9. Applications:

a.Wireless vibration monitoring
b.Remote structural vibration monitoring
c.Educational IoT and embedded systems projects
d.Experimental earthquake monitoring prototypes
e.Wireless sensor network demonstrations

11. Project Limitations:

This is an experimental vibration-monitoring prototype. It detects vibration conditions but does not predict earthquakes or provide certified earthquake early-warning services.

