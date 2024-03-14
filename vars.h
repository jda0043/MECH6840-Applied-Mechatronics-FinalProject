


#define Mot1S  4  
#define motorInterfaceType 1

const int DIR = 12;
const int STEP = 14;
const int DIR_2 = 27;
const int STEP_2 = 26;


const char* ssid = "PiNet";
const char* password = "quietrosebud964";
//const char* ssid = "Team -2";
//const char* password = "password";


String serverMessage = "";
String serverMessage_temp = "";
String message = "";
String Kp = "3.6";
String Ki = "0";
String Kd = "0.25";
String Kp_heading = "2";
String Ki_heading = "0";
String Kd_heading = "0";
unsigned long MAIN_LOOP_TIME = 10; // (ms) : 5ms=200hs 10ms=100hz 20ms=50hz
unsigned long MAIN_LOOP_TIME_HEADING = 20; // (ms) : 5ms=200hs 10ms=100hz 20ms=50hz
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

float leftMotor_speed = 0;
float rightMotor_speed = 0;
float motor_MAXspeed = 1000;

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

bool sensor_state = false;
unsigned long previous_time = 0; 
unsigned long reading_delay  = 50;
