


#define Mot1S  4  
#define motorInterfaceType 1


#define GAIN          0.5     /* Fusion gain, value between 0 and 1 - Determines orientation correction with respect to gravity vector. 
                                 If set to 1 the gyroscope is dissabled. If set to 0 the accelerometer is dissabled (equivant to gyro-only) */
#define SD_ACCEL      0.2  

const int DIR = 12;
const int STEP = 14;
const int DIR_2 = 27;
const int STEP_2 = 26;

const int MS1_1 = 25;
const int MS2_1 = 33;
const int MS3_1 = 32;
const int MS1_2 = 35;
const int MS2_2 = 34;
const int MS3_2 = 39;

const int MOT_1_ENABLE = 0;
const int MOT_2_ENABLE = 2;

const int stepsPerRevolution = 200;

int stepCount = 0;         

unsigned long fusiontimestep = 0;
unsigned long previous_fusiontimestep = 0;


//const char* ssid = "PiNet";
//const char* password = "quietrosebud964";
const char* ssid = "Team-2";
const char* password = "password";


String serverMessage = "";
String serverMessage_temp = "";
String message = "";
String Kp = "3.6";
String Ki = "0";
String Kd = "0";
String Kp_heading = "1";
String Ki_heading = "0";
String Kd_heading = "0";
unsigned long MAIN_LOOP_TIME = 10; // (ms) : 5ms=200hz 10ms=100hz 20ms=50hz
unsigned long MAIN_LOOP_TIME_HEADING = 20; // (ms) : 5ms=200hz 10ms=100hz 20ms=50hz
unsigned long SENSE_LOOP_TIME = 2; // (ms) 
int dutyCycle1 = 0;

int controlLoop_bool = false;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float heading = 0.0; 
float pitch = 0;
float roll = 0;

float setpoint = 0;
float heading_setpoint = 0;

float pitch_setpoint_offset = 0;
float heading_setpoint_offset = 0;

long leftMotor_speed = 0;
long rightMotor_speed = 0;
long motor_MAXspeed = 20000;
long motor_MAXacceleration = 10000;

float DEADBAND = 0.1;

// Pitch Controller 
float error = 0;
float previous_error = 0;
float total_error = 0;
float delta_error = 0;
unsigned long previous_control_time = 0;

unsigned long period_T = 10;

// Heading Controller
float error_heading = 0;
float previous_error_heading = 0;
float total_error_heading = 0;
float delta_error_heading = 0;
unsigned long previous_control_time_heading = 0;

unsigned long period_T_heading = 10;

bool sensor_state = true;
unsigned long previous_time = 0; 
unsigned long reading_delay  = 100;
