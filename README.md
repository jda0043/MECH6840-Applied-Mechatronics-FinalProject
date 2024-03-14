**Project Software for MECH 6840 - Applied Mechatronics Team -2.
**
Architecture - ESP32 WROOM


**Required Libraries**
1. WiFi ..................... Arduino WiFi Library
2. HTTPClient ............... 
3. AsyncTCP .................
4. ESPAsyncWebServer ........ 
5. LittleFS ................. Filesystem Handling/Uploader - used to handle and upload server html, css, and javascript files
6. Arduino_JSON ............. JSON Library for creating JSON variables - used for websocket messages
7. ArduinoOTA ............... Library for uploading scripts to microcontroller OTA
8. AsyncElegantOTA .......... Additional library for OTA uploads, this includes filesystem
9. Wire ..................... Wire Library for I2C communication
10. MPU9250 ................. Library for the MPU9250 communications and calibrations
11. imuFilter ............... Filters gyro and accelerometer values through rotations for yaw, pitch, and roll.
12. AccelStepper ............ Stepper Library used to control stepper motor drivers

**Pinout**
ESP32 GPIO 12 - A4988 DIRECTION
ESP32 GPIO 14 - A4988 STEP
ESP32 GPIO 27 - A4988 DIRECTION
ESP32 GPIO 26 - A4988 STEP

ESP32 GPIO SCL - MPU SCL
ESP32 GPIO SDA - MPU SDA


