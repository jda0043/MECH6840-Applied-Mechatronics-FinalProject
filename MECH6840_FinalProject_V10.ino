
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <Arduino_JSON.h>

#include <ArduinoOTA.h>
#include <AsyncElegantOTA.h>

#include "fastStepper.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"




#include <Adafruit_BNO08x.h>

#include "vars.h"


Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void IRAM_ATTR motLeftTimerFunction();
void IRAM_ATTR motRightTimerFunction();


fastStepper motLeft(14, 12, 0, motLeftTimerFunction);
fastStepper motRight(26, 27, 1, motRightTimerFunction);

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR motLeftTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motLeft.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}
void IRAM_ATTR motRightTimerFunction() {
  portENTER_CRITICAL_ISR(&timerMux);
  motRight.timerFunction();
  portEXIT_CRITICAL_ISR(&timerMux);
}




AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//Json Variable to Hold Slider Values
JSONVar sliderValues;



//Get Slider Values
String getSliderValues(){
  sliderValues["sliderValue1"] = Kp;
  sliderValues["sliderValue2"] = Ki;
  sliderValues["sliderValue3"] = Kd;
  sliderValues["sliderValue4"] = Kp_heading;
  sliderValues["sliderValue5"] = Kd_heading;
  sliderValues["sliderValue6"] = Kp_s;
  sliderValues["sliderValue7"] = Ki_s;
  sliderValues["sliderValue8"] = Kd_s;
  
  sliderValues["sliderValue9"] = String(DEADBAND);
  sliderValues["sliderValue0"] = String(DEADBAND_SPEED);
  
  sliderValues["sliderValue11"] = String(MAIN_LOOP_TIME);
  sliderValues["sliderValue12"] = String(MAIN_LOOP_TIME_HEADING);
  sliderValues["sliderValue13"] = String(MAIN_LOOP_TIME_SPEED);
  sliderValues["sliderValue14"] = String(reading_delay);
  
  sliderValues["heading"] = String(heading);
  sliderValues["pitch"] = String(pitch);
  sliderValues["roll"] = String(roll);
  sliderValues["gZ"] = String(gyroZ);
  sliderValues["aX"] = String(accelerationX);
  sliderValues["LMSpeed"] = String(motLeft.speed);
  sliderValues["RMSpeed"] = String(motRight.speed);
  sliderValues["ConEN"] = String(enCon);
  sliderValues["MotEN"] = String(enMot);
  sliderValues["SetpointP"] = String(setpoint);
  sliderValues["SetpointS"] = String(speed_setpoint);
  String jsonString = JSON.stringify(sliderValues);
  return jsonString;
  
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
      Kp_heading = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("5s") >= 0) {
      Kd_heading = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("6s") >= 0) {
      Kp_s = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("7s") >= 0) {
      Ki_s = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("8s") >= 0) {
      Kd_s = message.substring(2);
      notifyClients(getSliderValues());
    }
    if (message.indexOf("9s") >= 0) {
      DEADBAND = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("0s") >= 0) {
      DEADBAND_SPEED = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("!s") >= 0) {
      MAIN_LOOP_TIME = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("@s") >= 0) {
      MAIN_LOOP_TIME_HEADING = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("#s") >= 0) {
      MAIN_LOOP_TIME_SPEED = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }
    if (message.indexOf("$s") >= 0) {
      reading_delay = message.substring(2).toFloat();
      notifyClients(getSliderValues());
    }


    if (message.indexOf("SP") >= 0) {
      int ind1 = message.indexOf(",");
      speed_setpoint = message.substring(2,ind1).toFloat();
      int ind2 = message.indexOf(",", ind1+1);
      heading_setpoint = message.substring(ind1+1, ind2).toFloat();
    }
    
    
    if (strcmp((char*)data, "ENCON") == 0) {
      controlLoop_bool = true;
      enCon = 1;
    }
    if (strcmp((char*)data, "DISCON") == 0) {
      controlLoop_bool = false;
      enCon = 0;
    }
    
    if (strcmp((char*)data, "ENMOT") == 0) {
      digitalWrite(MOT_1_ENABLE, LOW);
      digitalWrite(MOT_2_ENABLE, LOW); 
      enMot = 1;
    }
    if (strcmp((char*)data, "DISMOT") == 0) {
      digitalWrite(MOT_1_ENABLE, HIGH);
      digitalWrite(MOT_2_ENABLE, HIGH); 
      enMot = 0;
    }
    if (strcmp((char*)data, "UD") == 0) {
       Kp = "2.5";
       Ki = "0.01";
       Kd = "5";
       notifyClients(getSliderValues());
    }
    if (strcmp((char*)data, "CD") == 0) {
       Kp = "6.5";
       Ki = "0.01";
       Kd = "12";
       notifyClients(getSliderValues());
    }

    if (strcmp((char*)data, "OD") == 0) {
       Kp = "14";
       Ki = "0.08";
       Kd = "8";
       notifyClients(getSliderValues());
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
   delay(100);
   pinMode(MS1_1, OUTPUT);
   pinMode(MS2_1, OUTPUT);
   pinMode(MS3_1, OUTPUT);
   delay(100);
   pinMode(MOT_1_ENABLE, OUTPUT);
   pinMode(MOT_2_ENABLE, OUTPUT);
   delay(100);
   digitalWrite(MOT_1_ENABLE, HIGH);
   digitalWrite(MOT_2_ENABLE, HIGH);

   digitalWrite(MS1_1, HIGH); 
   digitalWrite(MS2_1, HIGH);  
   digitalWrite(MS3_1, HIGH); 


   delay(1000);
   bno08x.begin_I2C();
   delay(250);
   bno08x.enableReport(reportType, reportIntervalUs);
   delay(250);
   bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED);
   delay(500);
  
   //LittleFS.begin();

   delay(100);
   WiFi.softAP(ssid, password);
   //WiFi.begin(ssid, password);
   initWebSocket();
   //IPAddress IP = WiFi.localIP();
   //Serial.println(IP);
   delay(100);
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
  delay(100);
  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  delay(100);
  
  server.begin();

  //server.serveStatic("/", LittleFS, "/");

  // Web Server Root URL
  //server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    //request->send(LittleFS, "/index.html", "text/html");
  //});
  
  delay(100);
  setMicroStep(microStep);
  motLeft.init();
  motRight.init();
  motLeft.microStep = microStep;
  motRight.microStep = microStep;


}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);
    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {

    if (bno08x.getSensorEvent(&sensorValue)) {
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            gyroZ = sensorValue.un.gyroscope.z;
            break;
        case SH2_LINEAR_ACCELERATION:
            accelerationX = sensorValue.un.linearAcceleration.x;
            break;
     } 
      heading = ypr.yaw;
      pitch = ypr.pitch;
      roll = ypr.roll;         
    }
    
    
    if (controlLoop_bool == true && abs(setpoint-roll) >= DEADBAND && abs(roll) < 40) {
     control_loop(-roll);
     if (abs(speed_setpoint-leftMotor_speed) >= DEADBAND_SPEED) {
        control_loop_speed(leftMotor_speed);
     }
     else{
        control_loop_speed(0);
     }
     if (abs(heading_setpoint-gyroZ) >= DEADBAND_HEADING) {
        control_loop_heading(-gyroZ); 
     }   
    }
    else {
      current_time = millis();
      motLeft.speed = 0;
      motRight.speed = 0;
      total_error = 0;
      total_error_heading = 0;
      total_error_speed = 0;
      previous_error = 0;
      previous_error_heading = 0;
      previous_error_speed = 0;
      previous_control_time = current_time;
      previous_control_time_heading = current_time;
      previous_control_time_speed = current_time;
    }
  
    if (sensor_state == true && (millis() - previous_time) > reading_delay ) {
      notifyClients(getSliderValues());
      previous_time = millis();
    }

    

    

  
    motLeft.update();
    motRight.update();

    ArduinoOTA.handle();
    ws.cleanupClients();

}



void control_loop(float actual_heading) {
  current_time = millis();
  delta_time = current_time - previous_control_time; 
  if (delta_time >= MAIN_LOOP_TIME) {
    error = setpoint_static - actual_heading - setpoint;
    total_error += error;
    delta_error = error - previous_error;

    long kp_term = error * Kp.toFloat();
    long kd_term = (delta_error / delta_time) * Kd.toFloat();
    long ki_term = (total_error * delta_time) * Ki.toFloat();
    
    leftMotor_speed = kp_term + kd_term + ki_term;
    rightMotor_speed = kp_term + kd_term + ki_term;

    motLeft.speed = leftMotor_speed;
    motRight.speed = -leftMotor_speed;

    // Switch microstepping
    avgMotSpeedSum += leftMotor_speed/2;
    if (avgMotSpeedSum>maxStepSpeed) {
        avgMotSpeedSum  = maxStepSpeed;
    } 
    else if (avgMotSpeedSum<-maxStepSpeed) {
        avgMotSpeedSum  = -maxStepSpeed;
    }

    leftMotor_speed = avgMotSpeedSum;
    absSpeed = abs(avgMotSpeed);
    uint8_t lastMicroStep = microStep;

    if (absSpeed > (150 * 32 / microStep) && microStep > 1) microStep /= 2;
    if (absSpeed < (130 * 32 / microStep) && microStep < 32) microStep *= 2;


    previous_error = error;
    previous_control_time = current_time;
  }
}




void control_loop_heading(float actual_heading) {
  current_time_heading = millis();
  delta_time_heading = current_time_heading - previous_control_time_heading; 
  if (delta_time_heading >= MAIN_LOOP_TIME_HEADING ) {
    error_heading = heading_setpoint - actual_heading;
    total_error_heading += error_heading;
    delta_error_heading = error_heading - previous_error_heading;
    
    kp_term_heading = error_heading * Kp_heading.toFloat();
    kd_term_heading = (delta_error_heading / delta_time_heading) * Kd_heading.toFloat();
    
    //leftMotor_speed -= kp_term_heading + kd_term_heading;
    //rightMotor_speed -=  kp_term_heading + kd_term_heading;
    motLeft.speed += kp_term_heading + kd_term_heading;
    motRight.speed += kp_term_heading + kd_term_heading;

    previous_error_heading = error_heading;
    previous_control_time_heading = current_time_heading;
   }
}


void control_loop_speed(float actual_speed) {
  current_time_speed = millis();
  delta_time_speed = current_time_speed - previous_control_time_speed; 
  if (delta_time_speed >= MAIN_LOOP_TIME_SPEED ) {
    error_speed = speed_setpoint - actual_speed;
    total_error_speed += error_speed;
    delta_error_speed = error_speed - previous_error_speed;
    
    kp_term_speed = error_speed * Kp_s.toFloat();
    kd_term_speed = (delta_error_speed / delta_time_speed) * Kd_s.toFloat();
    ki_term_speed = (total_error_speed * delta_time_speed) * Ki_s.toFloat();
    
    setpoint = kp_term_speed + kd_term_speed + ki_term_speed;

    if (setpoint > 35) {
      setpoint = 35;
    }
    if (setpoint < -35) {
      setpoint = -35;
    }

    previous_error_speed = error_speed;
    previous_control_time_speed = current_time_speed;
   }
}





void setMicroStep(uint8_t uStep) {
  // input:                     1 2 4 8 16 32
  // uStep table corresponds to 0 1 2 3 4  5  in binary on uStep pins
  // So, we need to take the log2 of input
  uint8_t uStepPow = 0;
  uint8_t uStepCopy = uStep;
  while (uStepCopy >>= 1) uStepPow++;

  if (uStep==16) {
    digitalWrite(MS1_1, 1); 
    digitalWrite(MS2_1, 1); 
    digitalWrite(MS3_1, 1); 
  }  
}
