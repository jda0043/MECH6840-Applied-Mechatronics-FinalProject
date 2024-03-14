
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncElegantOTA.h>

#include <SPI.h>
#include <Wire.h>

#include "MPU9250.h"
#include <imuFilter.h>

#include <AccelStepper.h>

#include "vars.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
imuFilter fusion;

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

AccelStepper stepper(motorInterfaceType, STEP, DIR);

AccelStepper stepper_2(motorInterfaceType, STEP_2, DIR_2);

//Json Variable to Hold Slider Values
JSONVar sliderValues;
JSONVar readings;

//Get Slider Values
String getSliderValues(){
  sliderValues["sliderValue1"] = String(Kp);
  sliderValues["sliderValue2"] = String(Ki);
  sliderValues["sliderValue3"] = String(Kd);
  sliderValues["heading"] = String(heading);
  sliderValues["pitch"] = String(rightMotor_speed);
  sliderValues["roll"] = String(roll);
  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
  
}

// Initialize LittleFS
void initFS() {
  if (!LittleFS.begin()) {
    //Serial.println("An error has occurred while mounting LittleFS");
  }
  else{
    //Serial.println("LittleFS mounted successfully");
  }
}


void notifyClients(String sliderValues) {
  ws.textAll(sliderValues);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    message = (char*)data;
    Serial.println(message);
    if (message.indexOf("1s") >= 0) {
      Kp = message.substring(2);
      //dutyCycle1 = map(speedVal.toInt(), 0, 100, minPWM, maxPWM);
      //dutyCycle4 = dutyCycle1;
      notifyClients(getSliderValues());
    }
    if (message.indexOf("2s") >= 0) {
      Ki = message.substring(2);
      //headVal = message.substring(2);
      //headSetpoint = map(headVal.toInt(), 0, 100, 0, 180);
      notifyClients(getSliderValues());
    }    
    if (message.indexOf("3s") >= 0) {
      Ki = message.substring(2);
      //liftVal = message.substring(2);
      //liftSetpoint = map(liftVal.toInt(), 0, 100, 0, 255);
      notifyClients(getSliderValues());
    }
    if (strcmp((char*)data, "SENSORS") == 0) {
      if (sensor_state == false) {
        sensor_state = true;
      }
      else {
        sensor_state = false;
      }
    }
    if (strcmp((char*)data, "FWD") == 0) {
      //stepper.setSpeed(-200);  
      //stepper_2.setSpeed(200);  
      //setpoint += 4;
      heading_setpoint += 20;
    }
    if (strcmp((char*)data, "BWD") == 0) {
      //stepper.setSpeed(200);  
      //stepper_2.setSpeed(-200); 
      setpoint -= 6; 
    }
    if (strcmp((char*)data, "stop") == 0) {
      //stepper.setSpeed(0);
      //stepper_2.setSpeed(0);  
      setpoint = 0;  
    }
    if (strcmp((char*)data, "LB") == 0) {
      controlLoop_bool = false;
      stepper.setSpeed(0);
      stepper_2.setSpeed(0);
      //setpoint = heading;   
    }
    if (strcmp((char*)data, "RB") == 0) {
      controlLoop_bool = true;
    }
    if (strcmp((char*)data, "DashBoard") == 0) {

    }
    if (strcmp((char*)data, "getValues") == 0) {
      notifyClients(getSliderValues());
    }
  }
}


void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      //Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      //Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}



void setup() {
   Serial.begin(115200);
   
   // start communication with IMU
   status = IMU.begin();
   IMU.calibrateGyro();
   delay(100);
   //IMU.calibrateMag();
   //delay(100);
   IMU.calibrateAccel();
   // start the IMU and filter
                     // Set DMP FIFO rate to 10 Hz

   fusion.setup( IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss() ); 
   float angle = 0 * DEG_TO_RAD;                // angle in radians to rotate heading about z-axis
   fusion.rotateHeading( angle, LARGE_ANGLE ); 

   initFS();

   //WiFi.softAP(ssid, password);
   WiFi.begin(ssid, password);
   initWebSocket();
   IPAddress IP = WiFi.localIP();
   Serial.println(IP);

   ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else 
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      //Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      //Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      //Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) ;//Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) ;//Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) ;//Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) ;//Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) ;//Serial.println("End Failed");
    });

  ArduinoOTA.begin();
  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  
  server.begin();

  server.serveStatic("/", LittleFS, "/");

  // Web Server Root URL
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  stepper.setMaxSpeed(motor_MAXspeed);
  stepper_2.setMaxSpeed(motor_MAXspeed);
  stepper.setSpeed(0); 
  stepper_2.setSpeed(0); 
  
}

void loop() {
  uint32_t currentMillis = millis();
  IMU.readSensor();
  accelX = IMU.getAccelX_mss();
  accelY = IMU.getAccelY_mss();
  accelZ = IMU.getAccelZ_mss();
  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();
  fusion.update( IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss() ); 
  heading = fusion.yaw() * (180/PI);
  pitch = fusion.pitch() * (180/PI);
  roll = fusion.roll() * (180/PI);

  if (controlLoop_bool == true) {
   control_loop(roll);
   control_loop_heading(heading); 
  }

  if (sensor_state == true && (millis() - previous_time) > reading_delay ) {
    notifyClients(getSliderValues());
    previous_time = millis();
  }

 stepper.runSpeed();
 stepper_2.runSpeed();
  
  
  ArduinoOTA.handle();
  ws.cleanupClients();

}



void control_loop(float actual_heading) {
  unsigned long current_time = millis();
  unsigned long delta_time = current_time - previous_control_time; 
  if (delta_time >= MAIN_LOOP_TIME) {
    error = setpoint - actual_heading;
    total_error += error;
    delta_error = error - previous_error;
    
    float kp_term = error * Kp.toFloat() * 10;
    float kd_term = (delta_error / MAIN_LOOP_TIME) * Kd.toFloat() * 0.001;
    float ki_term = (total_error * MAIN_LOOP_TIME) * Ki.toFloat();
  
    leftMotor_speed = kp_term + kd_term + ki_term;
    rightMotor_speed = kp_term + kd_term + ki_term;
    if (leftMotor_speed > 750) {
      leftMotor_speed = 750;
    }
    if (leftMotor_speed < -750) {
      leftMotor_speed = -750;
    }
  
  
    previous_error = error;
    previous_control_time = current_time;

  
    stepper.setSpeed(-leftMotor_speed);
    stepper_2.setSpeed(leftMotor_speed);
  }
}




void control_loop_heading(float actual_heading) {
  unsigned long current_time_heading = millis();
  unsigned long delta_time_heading = current_time_heading - previous_control_time_heading; 
  if (delta_time_heading >= MAIN_LOOP_TIME_HEADING) {
    error_heading = heading_setpoint - actual_heading;
    total_error_heading += error_heading;
    delta_error_heading = error_heading - previous_error_heading;
    
    float kp_term = error_heading * Kp_heading.toFloat();
    float kd_term = (delta_error_heading / MAIN_LOOP_TIME_HEADING) * Kd_heading.toFloat();
    float ki_term = (total_error_heading * MAIN_LOOP_TIME_HEADING) * Ki_heading.toFloat();
  
    leftMotor_speed += kp_term + kd_term + ki_term;
    rightMotor_speed -= kp_term + kd_term + ki_term;
    if (leftMotor_speed > 750) {
      leftMotor_speed = 750;
    }
    if (leftMotor_speed < -750) {
      leftMotor_speed = -750;
    }
  
  
    previous_error_heading = error_heading;
    previous_control_time_heading = current_time_heading;
  
    stepper.setSpeed(-rightMotor_speed);
    stepper_2.setSpeed(leftMotor_speed);
  }
}
