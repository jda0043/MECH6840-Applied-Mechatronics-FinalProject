
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>

#include <ArduinoOTA.h>
#include <AsyncElegantOTA.h>


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
  sliderValues["sliderValue4"] = String(pitch_setpoint_offset);
  sliderValues["sliderValue5"] = String(Kp_heading);
  sliderValues["sliderValue6"] = String(Ki_heading);
  sliderValues["sliderValue7"] = String(Kd_heading);
  sliderValues["sliderValue8"] = String(heading_setpoint_offset);
  sliderValues["sliderValue9"] = String(DEADBAND);
  sliderValues["sliderValue10"] = String(MAIN_LOOP_TIME);
  sliderValues["sliderValue11"] = String(MAIN_LOOP_TIME_HEADING);
  sliderValues["sliderValue12"] = String(SENSE_LOOP_TIME);
  sliderValues["heading"] = String(heading);
  sliderValues["pitch"] = String(pitch);
  sliderValues["roll"] = String(roll);
  sliderValues["gx"] = String(gyroX);
  sliderValues["gy"] = String(gyroY);
  sliderValues["gz"] = String(gyroZ);
  sliderValues["ax"] = String(accelX);
  sliderValues["ay"] = String(setpoint);
  sliderValues["az"] = String(heading_setpoint);
  sliderValues["LMSpeed"] = String(leftMotor_speed);
  sliderValues["RMSpeed"] = String(rightMotor_speed);
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
      notifyClients(getSliderValues());
    }
    if (message.indexOf("2s") >= 0) {
      Ki = message.substring(2);
      notifyClients(getSliderValues());
    }    
    if (message.indexOf("3s") >= 0) {
      Kd = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("4s") >= 0) {
      pitch_setpoint_offset = message.substring(2).toFloat();
      notifyClients(getSliderValues());
      setpoint = pitch_setpoint_offset;
    }
    if (message.indexOf("5s") >= 0) {
      Kp_heading = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("6s") >= 0) {
      Ki_heading = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("7s") >= 0) {
      Kd_heading = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("8s") >= 0) {
      heading_setpoint_offset = message.substring(2).toFloat();
      notifyClients(getSliderValues());
      heading_setpoint = heading_setpoint_offset;
    }
    if (message.indexOf("9s") >= 0) {
      DEADBAND = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("0s") >= 0) {
      MAIN_LOOP_TIME = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("!s") >= 0) {
      MAIN_LOOP_TIME_HEADING = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("#s") >= 0) {
      SENSE_LOOP_TIME = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }

    if (message.indexOf("SP") >= 0) {
      int ind1 = message.indexOf(",");
      setpoint = message.substring(2,ind1).toFloat();
      int ind2 = message.indexOf(",", ind1+1);
      float heading_setpoint_temp = message.substring(ind1+1, ind2).toFloat();

      setpoint = map(setpoint,-50, 50, -12, 12) + pitch_setpoint_offset;
      //heading_setpoint = map(heading_setpoint_temp,-50, 50, -180, 180) + heading_setpoint_offset;
      heading_setpoint += heading_setpoint_temp;
      if (heading_setpoint > 179) {
        heading_setpoint = 179;
      }
      if (heading_setpoint < -179) {
        heading_setpoint = -179;
      }
      
    }
    
    
    if (strcmp((char*)data, "ENCON") == 0) {
      controlLoop_bool = true;
      previous_control_time = millis();
      previous_control_time_heading = millis();
    }
    if (strcmp((char*)data, "DISCON") == 0) {
      controlLoop_bool = false;
      stepper.setSpeed(0); 
      stepper_2.setSpeed(0); 
    }
    
    if (strcmp((char*)data, "ENMOT") == 0) {
      digitalWrite(MOT_1_ENABLE, LOW);
      digitalWrite(MOT_2_ENABLE, LOW); 
    }
    if (strcmp((char*)data, "DISMOT") == 0) {
      digitalWrite(MOT_1_ENABLE, HIGH);
      digitalWrite(MOT_2_ENABLE, HIGH); 
    }
    if (strcmp((char*)data, "WS") == 0) {
       digitalWrite(MS1_1, LOW); 
       digitalWrite(MS2_1, LOW);  
       digitalWrite(MS3_1, LOW); 
       digitalWrite(MS1_2, LOW);  
       digitalWrite(MS2_2, LOW);
       digitalWrite(MS3_2, LOW);
    }
    if (strcmp((char*)data, "HS") == 0) {
       //digitalWrite(MS1_1, HIGH); 
       //digitalWrite(MS2_1, LOW);  
       //digitalWrite(MS3_1, LOW); 
       //digitalWrite(MS1_2, HIGH);  
       //digitalWrite(MS2_2, LOW);
       //digitalWrite(MS3_2, LOW);
    }
    if (strcmp((char*)data, "QS") == 0) {
       digitalWrite(MS1_1, LOW); 
       digitalWrite(MS2_1, HIGH);  
       digitalWrite(MS3_1, LOW); 
       digitalWrite(MS1_2, LOW);  
       digitalWrite(MS2_2, HIGH);
       digitalWrite(MS3_2, LOW);
    }
    if (strcmp((char*)data, "ES") == 0) {
       //digitalWrite(MS1_1, HIGH); 
       //digitalWrite(MS2_1, HIGH);  
       //digitalWrite(MS3_1, LOW); 
       //digitalWrite(MS1_2, HIGH);  
       //digitalWrite(MS2_2, HIGH);
       //digitalWrite(MS3_2, LOW);
    }
    if (strcmp((char*)data, "SS") == 0) {
       //digitalWrite(MS1_1, HIGH); 
       //digitalWrite(MS2_1, HIGH);  
       //digitalWrite(MS3_1, HIGH); 
       //digitalWrite(MS1_2, HIGH);  
       //digitalWrite(MS2_2, HIGH);
       //digitalWrite(MS3_2, HIGH);
    }
    
    if (strcmp((char*)data, "CIMU") == 0) {
      //IMU.calibrateGyro();
      //delay(100);
      //IMU.calibrateAccel();
      //delay(100);
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
   pinMode(MS1_1, OUTPUT);
   pinMode(MS2_1, OUTPUT);
   pinMode(MS3_1, OUTPUT);
   pinMode(MS1_2, OUTPUT);
   pinMode(MS2_2, OUTPUT);
   pinMode(MS3_2, OUTPUT);
   pinMode(MOT_1_ENABLE, OUTPUT);
   pinMode(MOT_2_ENABLE, OUTPUT);

   digitalWrite(MOT_1_ENABLE, HIGH);
   digitalWrite(MOT_2_ENABLE, HIGH);

   digitalWrite(MS1_1, HIGH); 
   digitalWrite(MS2_1, HIGH);  
   digitalWrite(MS3_1, HIGH); 
   digitalWrite(MS1_2, HIGH);  
   digitalWrite(MS2_2, HIGH);
   digitalWrite(MS3_2, HIGH);   
     
   
   // start communication with IMU
   status = IMU.begin();
   IMU.calibrateGyro();
   delay(100);
   //IMU.calibrateMag();
   //delay(100);
   IMU.calibrateAccel();
   delay(100);
   // start the IMU and filter
                     // Set DMP FIFO rate to 10 Hz

   fusion.setup( IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss() ); 
   //fusion.setup();     
   float angle = 0 * DEG_TO_RAD;                // angle in radians to rotate heading about z-axis
   fusion.rotateHeading( angle, LARGE_ANGLE ); 

   initFS();

   WiFi.softAP(ssid, password);
   //WiFi.begin(ssid, password);
   initWebSocket();
   //IPAddress IP = WiFi.localIP();
   //Serial.println(IP);

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
  stepper.setAcceleration(motor_MAXacceleration);
  stepper.setAcceleration(motor_MAXacceleration);
  stepper.setSpeed(0); 
  stepper_2.setSpeed(0); 
  
}

void loop() {
  fusiontimestep = millis() - previous_fusiontimestep;
  if (fusiontimestep >= SENSE_LOOP_TIME) {
  //uint32_t currentMillis = millis();
  IMU.readSensor();
  //accelX = IMU.getAccelX_mss();
  //accelY = IMU.getAccelY_mss();
  //accelZ = IMU.getAccelZ_mss();
  //gyroX = IMU.getGyroX_rads();
  //gyroY = IMU.getGyroY_rads();
  //gyroZ = IMU.getGyroZ_rads();
  fusion.update( IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), GAIN, SD_ACCEL );
  //fusion.update( IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads() );
  heading = fusion.yaw() * (180/PI);
  pitch = fusion.pitch() * (180/PI);
  roll = fusion.roll() * (180/PI);

  if (controlLoop_bool == true && abs(roll) < 30 && abs(heading_setpoint-roll) >= DEADBAND) {
   control_loop(roll);
   control_loop_heading(heading); 
  }
  else {
    stepper.setSpeed(0);
    stepper_2.setSpeed(0);
  }

  if (sensor_state == true && (millis() - previous_time) > reading_delay ) {
    notifyClients(getSliderValues());
    previous_time = millis();
  }

 stepper.runSpeed();
 stepper_2.runSpeed();

 //stepper.run();
 //stepper_2.run();
 previous_fusiontimestep = fusiontimestep;
  }


  ArduinoOTA.handle();
  ws.cleanupClients();

}



void control_loop(float actual_heading) {
  unsigned long current_time = millis();
  unsigned long delta_time = current_time - previous_control_time; 
  //fusiontimestep = delta_time;
  if (delta_time >= MAIN_LOOP_TIME) {
    error = setpoint - actual_heading;
    total_error += error;
    delta_error = error - previous_error;
    
    long kp_term = error * Kp.toFloat() * 160;
    long kd_term = (delta_error / MAIN_LOOP_TIME) * Kd.toFloat() *16;
    long ki_term = (total_error * MAIN_LOOP_TIME) * Ki.toFloat();
  
    leftMotor_speed = kp_term + kd_term + ki_term;
    rightMotor_speed = kp_term + kd_term + ki_term;

  
  
    previous_error = error;
    previous_control_time = current_time;

  
    stepper.setSpeed(-leftMotor_speed);
    stepper_2.setSpeed(leftMotor_speed);
    //stepper.moveTo(16);
    //stepper_2.moveTo(16);
  }
}




void control_loop_heading(float actual_heading) {
  unsigned long current_time_heading = millis();
  unsigned long delta_time_heading = current_time_heading - previous_control_time_heading; 
  if (delta_time_heading >= MAIN_LOOP_TIME_HEADING) {
    error_heading = heading_setpoint - actual_heading;
    total_error_heading += error_heading;
    delta_error_heading = error_heading - previous_error_heading;
    
    long kp_term = error_heading * Kp_heading.toFloat() * 160;
    long kd_term = (delta_error_heading / MAIN_LOOP_TIME_HEADING) * Kd_heading.toFloat()*16;
    long ki_term = (total_error_heading * MAIN_LOOP_TIME_HEADING) * Ki_heading.toFloat();
  
    leftMotor_speed += kp_term + kd_term + ki_term;
    rightMotor_speed -= kp_term + kd_term + ki_term;
 
  
  
    previous_error_heading = error_heading;
    previous_control_time_heading = current_time_heading;
  
    stepper.setSpeed(-rightMotor_speed);
    stepper_2.setSpeed(leftMotor_speed);
    //stepper.moveTo(16);
    //stepper_2.moveTo(16);
  }
}
