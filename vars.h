



#define BNO08X_RESET -1

// Two options
//    -SH2_ARVR_STABILIZED_RV : Max Freq = 250Hz - More Accurate
//    -SH2_GYRO_INTEGRATED_RV : Max Freq = 1000Hz - Less accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV; 
long reportIntervalUs = 4000; // 250Hz 

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;




float heading = 0.0;
float pitch = 0.0;
float roll = 0.0;
float gyroZ = 0.0;
float accelerationX = 0.0;



uint8_t microStep = 16;
uint8_t motorCurrent = 150;
float maxStepSpeed = 1500;

float avgMotSpeed;
float avgMotSpeedSum;
float absSpeed;


int enMot = 0;
int enCon = 0;


const int DIR = 12;
const int STEP = 14;
const int DIR_2 = 27;
const int STEP_2 = 26;

const int MS1_1 = 33;
const int MS2_1 = 32;
const int MS3_1 = 13;

const int MOT_1_ENABLE = 0;
const int MOT_2_ENABLE = 2;


//const char* ssid = "PiNet";
//const char* password = "quietrosebud964";
const char* ssid = "Team-2";
const char* password = "password";


String serverMessage = "";
String serverMessage_temp = "";
String message = "";


String Kp = "6.5";
String Ki = "0.01";
String Kd = "12";
String Kp_heading = "0.25";
String Kd_heading = "0.01";
String Kp_s = "0.003";
String Ki_s = "0.0";
String Kd_s = "0.1";



int controlLoop_bool = false;
bool sensor_state = true;

// Send Sensor Data Variables
unsigned long previous_time = 0; 
unsigned long reading_delay  = 500;


long leftMotor_speed = 0;
long rightMotor_speed = 0;



// Pitch Controller
float DEADBAND = 0.0;
unsigned long MAIN_LOOP_TIME = 10; // (ms) : 5ms=200hz 10ms=100hz 20ms=50hz
float setpoint = 0.0; 
float setpoint_static = 0.0; 
float error = 0;
float previous_error = 0;
float total_error = 0;
float delta_error = 0;
unsigned long previous_control_time = 0;
unsigned long current_time = 0;
unsigned long delta_time = 0;


// Heading Controller
float DEADBAND_HEADING = 3.0;
unsigned long MAIN_LOOP_TIME_HEADING = 25; // (ms) : 5ms=200hz 10ms=100hz 20ms=50hz
float heading_setpoint = 0.0;
float error_heading = 0;
float previous_error_heading = 0;
float total_error_heading = 0;
float delta_error_heading = 0;
unsigned long previous_control_time_heading = 0;
unsigned long current_time_heading = 0;
unsigned long delta_time_heading = 0;
long kp_term_heading = 0;
long kd_term_heading = 0;




// Speed Controller
float DEADBAND_SPEED = 0.0;
unsigned long MAIN_LOOP_TIME_SPEED = 10;
float speed_setpoint = 0.0;
float error_speed = 0.0;
float previous_error_speed = 0.0;
float total_error_speed = 0.0;
float delta_error_speed = 0.0;
unsigned long previous_control_time_speed = 0.0;
unsigned long current_time_speed = 0;
unsigned long delta_time_speed = 0;
long kp_term_speed = 0;
long kd_term_speed = 0;
long ki_term_speed = 0;
