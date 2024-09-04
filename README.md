**Project Software for MECH 6840 - Applied Mechatronics Team -2.**

Architecture - ESP32 WROOM

**Included Files**
- MECH6840_FinalProject.ino : Main Firmware file uploaded to ESP32.
- vars.h : File for storing all parameters used in firmware file.
- data folder : All files in this folder make up the interface. Uploaded via LittleFS to flash memory of ESP32
  - index.html : HTML file for interface
  - script.js  : Javascript file for interface. Used to send commands to controller.
  - style.css  : CSS styling. Used to group elements on interface
- joycon_controller.py : Python file used to enable a switch joycon for controlling robot setpoints.

**Required Libraries**
1. WiFi ..................................... Arduino WiFi Library
2. HTTPClient .........................
3. AsyncTCP ............................
4. ESPAsyncWebServer ..........
5. LittleFS ................................ Filesystem Handling/Uploader - used to handle and upload server html, css, and javascript files
6. Arduino_JSON .................... JSON Library for creating JSON variables - used for websocket messages
7. ArduinoOTA ......................... Library for uploading scripts to microcontroller OTA
8. AsyncElegantOTA ............... Additional library for OTA uploads, this includes filesystem
9. Wire ..................................... Wire Library for I2C communication
10. MPU9250 ............................ Library for the MPU9250 communications and calibrations
11. imuFilter ............................... Filters gyro and accelerometer values through rotations for yaw, pitch, and roll.
12. AccelStepper ....................... Stepper Library used to control stepper motor drivers

**Pinout**
- ESP32 GPIO 12 - A4988 DIRECTION
- ESP32 GPIO 14 - A4988 STEP
- ESP32 GPIO 27 - A4988 DIRECTION
- ESP32 GPIO 26 - A4988 STEP
- ESP32 GPIO SCL - MPU SCL
- ESP32 GPIO SDA - MPU SDA
- ESP32 GPIO 25 - MS1_1
- ESP32 GPIO 33 - MS2_1
- ESP32 GPIO 32 - MS3_1
- ESP32 GPIO 35 - MS1_2
- ESP32 GPIO 34 - MS2_2
- ESP32 GPIO 39 - MS3_2
- ESP32 GPIO 0 - ENABLE 1
- ESP32 GPIO 2 - ENABLE 2


**Part List**
- ESP32 WROOM
- 2x A4988 Stepper Motor Drivers
- MPU9250 IMU / BNO085 IMU / MPU6050
- Matek 6-30V step-down regulator set to 9V
- Matek 6-30V step-down regulator set to 5V
- 2x Nema 17 biploar 1.8 degree 1.5A Stepper Motors (42x42x38mm)
- R-Line 4S 650mah LIPO battery


  **Interface**
  <img width="1272" alt="Screen Shot 2024-03-18 at 12 42 12 PM" src="https://github.com/jda0043/MECH6840-Applied-Mechatronics-FinalProject/assets/90729752/f512fa65-deb8-4922-b2ea-5690339a3683">
