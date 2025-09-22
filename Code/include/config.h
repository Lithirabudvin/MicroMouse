#ifndef CONFIG_H
#define CONFIG_H

// === MOTOR PINS ===
const int AIN1 = 18;
const int AIN2 = 17;
const int PWMA = 16;
const int BIN1 = 21;
const int BIN2 = 22;
const int PWMB = 19;

// === SENSOR PINS ===
const int XSHUT_LEFT = 13;
const int XSHUT_RIGHT = 27;
const int XSHUT_FRONT = 14;

// === I2C CONFIGURATION ===
const int I2C_SDA_PIN = 15;
const int I2C_SCL_PIN = 4;
const int I2C_CLOCK_SPEED = 50000;

// === WIFI CONFIGURATION ===
const char* WIFI_SSID = "ABC";
const char* WIFI_PASSWORD = "zzom5037";
const int TCP_PORT = 23;

// === WALL FOLLOWING PARAMETERS ===
const int DESIRED_DISTANCE = 90;
const int BASE_SPEED = 80;
const int MAX_SPEED = 150;
const int FRONT_STOP_DISTANCE = 120;
const int FRONT_SLOW_DISTANCE = 180;
const int WALL_THRESHOLD = 200;
const int EMERGENCY_STOP_DISTANCE = 80;
const int CLOSE_WALL_DISTANCE = 50;

// === PID CONTROL PARAMETERS ===
const float PID_KP = 0.35;
const float PID_KI = 0.002;
const float PID_KD = 0.6;
const int PID_DEADBAND = 5;
const float PID_MAX_INTEGRAL = 15.0;

// === MOTION PROFILING ===
const int MAX_SPEED_CHANGE = 3;
const float HEADING_CORRECTION_GAIN = 0.12;

// === TURN PARAMETERS ===
const int TURN_SPEED = 60;
const int TURN_ACCELERATION = 10;
const float TURN_ANGLE_TOLERANCE = 3.0;
const unsigned long TURN_TIMEOUT_MS = 5000;
const int POSITIONING_SPEED = 60;
const unsigned long POSITIONING_TIMEOUT = 1500;

// === SENSOR FILTERING ===
const int SENSOR_FILTER_SIZE = 7;
const float SENSOR_FILTER_ALPHA = 0.3;
const int MAX_SENSOR_RANGE = 1000;

// === IMU PARAMETERS ===
const float GYRO_SENSITIVITY = 131.0;  // LSB per degree/sec
const float GYRO_DRIFT_ALPHA = 0.001;
const unsigned long STILL_TIME_THRESHOLD = 2000;
const float HEADING_TOLERANCE = 1.5;
const float MAX_HEADING_CORRECTION = 4.0;

// === TIMING PARAMETERS ===
const unsigned long DEBUG_INTERVAL = 100;
const unsigned long TURN_DEBUG_INTERVAL = 200;
const unsigned long CORNER_DEBUG_INTERVAL = 500;
const unsigned long CALIBRATION_SAMPLES = 6;
const float MAX_DT = 0.1;
const float MIN_DT = 0.01;
const unsigned long LOOP_DELAY_MS = 20;

// === TURN COMPLETION PARAMETERS ===
const float TURN_COMPLETION_FACTOR = 0.8;
const float TURN_TOLERANCE_WIDE = 10.0;

// === ADAPTIVE PID FACTORS ===
const float ADAPTIVE_PID_CLOSE_FACTOR = 1.8;
const float ADAPTIVE_PID_MEDIUM_FACTOR = 1.3;
const float ADAPTIVE_DERIVATIVE_CLOSE = 1.5;
const float ADAPTIVE_DERIVATIVE_MEDIUM = 1.2;
const int ADAPTIVE_DISTANCE_CLOSE = 70;
const int ADAPTIVE_DISTANCE_MEDIUM = 100;
const float MAX_PID_CORRECTION = 25.0;

// === WALL DETECTION ===
const float CORNER_OPEN_THRESHOLD = 2.0;
const float CORNER_CLOSE_FACTOR = 0.7;
const float SLOWDOWN_FACTOR_MAX = 0.6;
const float SLOWDOWN_MIN_SPEED = 0.4;

// === POSITIONING ===
const int POSITIONING_MIN_DISTANCE = 30;

// === STATES ===
enum RobotState { FOLLOWING, TURNING, POSITIONING, STOPPED };

// === DEBUG FLAGS ===
#define DEBUG_CORNER_DETECTION 1
#define DEBUG_TURN_PROGRESS 1
#define DEBUG_SENSOR_READINGS 1
#define DEBUG_NETWORK 1

#endif