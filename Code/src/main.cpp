#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>  // Add MPU6050 library

// TOF Sensors
VL53L0X sensorLeft;
VL53L0X sensorRight;
VL53L0X sensorFront;  
const int xshutLeft = 13;
const int xshutRight = 27;
const int xshutFront = 14;  

// MPU6050 Accelerometer
MPU6050 mpu;  // Create MPU6050 object

// Motor Driver Pins
const int AIN1 = 18;  // Left motor direction 1
const int AIN2 = 17;  // Left motor direction 2
const int PWMA = 16;  // Left motor speed
const int BIN1 = 21;  // Right motor direction 1
const int BIN2 = 22;  // Right motor direction 2
const int PWMB = 19;  // Right motor speed

// Encoder Pins 
const int encoderLeftA = 32;  
const int encoderLeftB = 33;
const int encoderRightA = 25; 
const int encoderRightB = 26;

// Encoder Counts
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Wall-Following Parameters
const int desiredDistance = 100;  // mm
const int baseSpeed = 150;        // PWM (0-255)
const int maxSpeed = 200;         
const int minSpeed = 80;          
const int frontStopDistance = 150;
const int correctionStrength = 3;

// Motor Balancing Parameters
const float INITIAL_BALANCE = 0.93f;    // Start with 7% right motor reduction
const int BALANCE_SAMPLES = 10;         // Number of samples to average
const int BALANCE_INTERVAL = 500;       // Balance check interval (ms)
const int MIN_DELTA_FOR_BALANCE = 10;   // Minimum encoder ticks to consider

// Motor Balance Variables
float motorBalance = INITIAL_BALANCE;
unsigned long lastBalanceTime = 0;

// MPU6050 Variables
int16_t ax, ay, az;  // Acceleration raw values
int16_t gx, gy, gz;  // Gyroscope raw values
float accelX, accelY, accelZ;  // Acceleration in g-forces
float gyroX, gyroY, gyroZ;     // Gyroscope in degrees/sec

// Forward declaration
void balanceMotors(int leftDist, int rightDist, int frontDist);
void readMPU6050();  // New function to read MPU6050

void setup() {
  Serial.begin(115200);
  Wire.begin(15, 4);
  Wire.setClock(100000);

  // Initialize MPU6050
  mpu.initialize();
  delay(100);
  
  // Verify MPU6050 connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }
  
  // Calibrate MPU6050 (optional but recommended)
  Serial.println("Calibrating MPU6050... keep robot still!");
  mpu.CalibrateAccel(6);  // 6 samples for calibration
  mpu.CalibrateGyro(6);
  Serial.println("Calibration complete!");

  // Initialize TOF Sensors
  pinMode(xshutLeft, OUTPUT);
  pinMode(xshutRight, OUTPUT);
  pinMode(xshutFront, OUTPUT);
  
  digitalWrite(xshutLeft, LOW);
  digitalWrite(xshutRight, LOW);
  digitalWrite(xshutFront, LOW);
  delay(50);

  // Initialize Left Sensor
  digitalWrite(xshutLeft, HIGH);
  delay(15);
  if(!sensorLeft.init()) { Serial.println("Left sensor failed!"); while(1); }
  sensorLeft.setAddress(0x30);
  sensorLeft.setTimeout(500);
  sensorLeft.startContinuous();
  
  // Initialize Right Sensor
  digitalWrite(xshutRight, HIGH);
  delay(15);
  if(!sensorRight.init()) { Serial.println("Right sensor failed!"); while(1); }
  sensorRight.setAddress(0x32);
  sensorRight.setTimeout(500);
  sensorRight.startContinuous();
  
  // Initialize Front Sensor
  digitalWrite(xshutFront, HIGH);
  delay(15);
  if(!sensorFront.init()) { Serial.println("Front sensor failed!"); while(1); }
  sensorFront.setAddress(0x34);
  sensorFront.setTimeout(500);
  sensorFront.startContinuous();

  // Motor Control Setup
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcSetup(0, 3000, 8);
  ledcSetup(1, 3000, 8);
  ledcAttachPin(PWMA, 0);
  ledcAttachPin(PWMB, 1);

  // Encoder Setup
  pinMode(encoderLeftA, INPUT_PULLUP);
  pinMode(encoderLeftB, INPUT_PULLUP);
  pinMode(encoderRightA, INPUT_PULLUP);
  pinMode(encoderRightB, INPUT_PULLUP);
  
  // Encoder Interrupts
  attachInterrupt(digitalPinToInterrupt(encoderLeftA), [] {
    if (digitalRead(encoderLeftA)) {
      leftEncoderCount += (digitalRead(encoderLeftB)) ? 1 : -1;
    } else {
      leftEncoderCount += (digitalRead(encoderLeftB)) ? -1 : 1;
    }
  }, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderRightA), [] {
    if (digitalRead(encoderRightA)) {
      rightEncoderCount += (digitalRead(encoderRightB)) ? -1 : 1;
    } else {
      rightEncoderCount += (digitalRead(encoderRightB)) ? 1 : -1;
    }
  }, CHANGE);

  Serial.println("System Ready with MPU6050!");
}

void loop() {
  // Read sensors
  int leftDist = sensorLeft.readRangeContinuousMillimeters();
  int rightDist = sensorRight.readRangeContinuousMillimeters();
  int frontDist = sensorFront.readRangeContinuousMillimeters();
  
  // Read MPU6050 accelerometer and gyroscope
  readMPU6050();
  
  // Handle sensor timeouts
  if (sensorLeft.timeoutOccurred() || leftDist > 1000) leftDist = 1000;
  if (sensorRight.timeoutOccurred() || rightDist > 1000) rightDist = 1000;
  if (sensorFront.timeoutOccurred() || frontDist > 1000) frontDist = 1000;

  // Wall-Following Control
  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;

  // Left wall following (priority)
  if (leftDist < desiredDistance * 2) {
    int error = leftDist - desiredDistance;
    rightSpeed += error / correctionStrength;
  } 
  // Right wall following (secondary)
  else if (rightDist < desiredDistance * 2) {
    int error = rightDist - desiredDistance;
    leftSpeed += error / correctionStrength;
  }

  // Front collision avoidance
  if (frontDist < frontStopDistance) {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  // Speed limiting
  leftSpeed = constrain(leftSpeed, minSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, minSpeed, maxSpeed);

  // Motor balancing - run at fixed intervals
  if (millis() - lastBalanceTime > BALANCE_INTERVAL) {
    balanceMotors(leftDist, rightDist, frontDist);
    lastBalanceTime = millis();
  }

  // Apply balanced speeds
  int balancedRightSpeed = rightSpeed * motorBalance;
  
  // Drive motors
  digitalWrite(AIN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, leftSpeed > 0 ? LOW : HIGH);
  ledcWrite(0, abs(leftSpeed));

  digitalWrite(BIN1, balancedRightSpeed > 0 ? LOW : HIGH);
  digitalWrite(BIN2, balancedRightSpeed > 0 ? HIGH : LOW);
  ledcWrite(1, abs(balancedRightSpeed));

  // Debug output with MPU6050 data
  Serial.printf("TOF: L=%3d F=%3d R=%3d | ", leftDist, frontDist, rightDist);
  Serial.printf("MOTOR: L=%3d R=%3d | ", leftSpeed, balancedRightSpeed);
  Serial.printf("ENC: L=%5ld R=%5ld | ", leftEncoderCount, rightEncoderCount);
  Serial.printf("ACC: X=%-5.1fg Y=%-5.1fg Z=%-5.1fg | ", accelX, accelY, accelZ);
  Serial.printf("GYRO: X=%-5.0f Y=%-5.0f Z=%-5.0f\n", gyroX, gyroY, gyroZ);

  delay(50);
}

void readMPU6050() {
  // Read raw accelerometer and gyroscope values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Convert to meaningful units
  // MPU6050 default sensitivity: ±2g range, 16384 LSB/g
  accelX = ax / 16384.0;
  accelY = ay / 16384.0; 
  accelZ = az / 16384.0;
  
  // MPU6050 default sensitivity: ±250°/s range, 131 LSB/°/s
  gyroX = gx / 131.0;
  gyroY = gy / 131.0;
  gyroZ = gz / 131.0;
}

void balanceMotors(int leftDist, int rightDist, int frontDist) {
  static long lastLeft = 0, lastRight = 0;
  static int samples = 0;
  static float sumRatio = 0;
  
  // Only balance when moving straight (no wall following)
  if(leftDist >= 1000 && rightDist >= 1000 && frontDist >= 150) {
    long leftDelta = leftEncoderCount - lastLeft;
    long rightDelta = rightEncoderCount - lastRight;
    
    if(leftDelta > MIN_DELTA_FOR_BALANCE && rightDelta > MIN_DELTA_FOR_BALANCE) {
      float instantRatio = (float)leftDelta / rightDelta;
      sumRatio += instantRatio;
      samples++;
      
      if(samples >= BALANCE_SAMPLES) {
        motorBalance *= (sumRatio / samples); // Apply averaged correction
        motorBalance = constrain(motorBalance, 0.8f, 1.2f);
        
        Serial.print("Motor balance adjusted to: ");
        Serial.println(motorBalance, 3);
        
        // Reset for next average
        samples = 0;
        sumRatio = 0;
      }
    }
  }
  
  lastLeft = leftEncoderCount;
  lastRight = rightEncoderCount;
}