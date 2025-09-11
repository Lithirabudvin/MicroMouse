#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>

// TOF Sensors
VL53L0X sensorLeft;
VL53L0X sensorRight;
VL53L0X sensorFront;  
const int xshutLeft = 13;
const int xshutRight = 27;
const int xshutFront = 14;  

// MPU6050 Accelerometer
MPU6050 mpu;

// Motor Driver Pins
const int AIN1 = 18;
const int AIN2 = 17;
const int PWMA = 16;
const int BIN1 = 21;
const int BIN2 = 22;
const int PWMB = 19;

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
const int baseSpeed = 40;         // Keep for tuning
const int maxSpeed = 100;         
const int minSpeed = 40;          
const int frontStopDistance = 150;

// PID Parameters - Refined for stability
const float Kp = 0.2;  // Further reduced for gentler response
const float Ki = 0.05; // Small integral to correct steady-state error
const float Kd = 2.5;  // Increased for stronger damping

// PID Variables
float previousError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

// Motor Balancing Parameters
const float INITIAL_BALANCE = 0.9f; // Adjusted lower to compensate right motor
const int BALANCE_SAMPLES = 5;
const int BALANCE_INTERVAL = 200;
const int MIN_DELTA_FOR_BALANCE = 10;

// Motor Balance Variables
float motorBalance = INITIAL_BALANCE;
unsigned long lastBalanceTime = 0;

// MPU6050 Variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;

// Heading correction variables
float heading = 0.0;
float targetHeading = 0.0;
bool isTurning = false;

// Sensor Smoothing
const int FILTER_SIZE = 5; // Increased for more smoothing
int leftDistHistory[FILTER_SIZE];
int rightDistHistory[FILTER_SIZE];
int frontDistHistory[FILTER_SIZE];
int filterIndex = 0;

// Wall Selection Hysteresis
const int HYSTERESIS_THRESHOLD = 20; // mm difference to switch walls
bool followingLeft = false;

// ...existing code...

// Function prototypes
void readMPU6050();
void updateHeading();
void applyHeadingCorrection(int &leftSpeed, int &rightSpeed);
void balanceMotors(int leftDist, int rightDist, int frontDist);

// ...existing code...


void setup() {
  Serial.begin(115200);
  Wire.begin(15, 4);
  Wire.setClock(50000);

  // Initialize MPU6050
  mpu.initialize();
  delay(100);
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }
  Serial.println("Calibrating MPU6050... keep robot still!");
  mpu.CalibrateAccel(6);
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

  digitalWrite(xshutLeft, HIGH);
  delay(15);
  if(!sensorLeft.init()) { Serial.println("Left sensor failed!"); while(1); }
  sensorLeft.setAddress(0x30);
  sensorLeft.setTimeout(500);
  sensorLeft.startContinuous();
  
  digitalWrite(xshutRight, HIGH);
  delay(15);
  if(!sensorRight.init()) { Serial.println("Right sensor failed!"); while(1); }
  sensorRight.setAddress(0x32);
  sensorRight.setTimeout(500);
  sensorRight.startContinuous();
  
  digitalWrite(xshutFront, HIGH);
  delay(15);
  if(!sensorFront.init()) { Serial.println("Front sensor failed!"); while(1); }
  sensorFront.setAddress(0x34);
  sensorFront.setTimeout(500);
  sensorFront.startContinuous();

  // Initialize sensor history
  for (int i = 0; i < FILTER_SIZE; i++) {
    leftDistHistory[i] = 1000;
    rightDistHistory[i] = 1000;
    frontDistHistory[i] = 1000;
  }

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

  lastTime = millis();
  Serial.println("System Ready with MPU6050!");
}

int getFilteredDistance(VL53L0X &sensor, int *history) {
  int raw = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || raw > 1000) raw = 1000;
  history[filterIndex % FILTER_SIZE] = raw;
  int sum = 0;
  int validCount = 0;
  for (int i = 0; i < FILTER_SIZE; i++) {
    if (history[i] < 1000) {
      sum += history[i];
      validCount++;
    }
  }
  return validCount > 0 ? sum / validCount : 1000;
}

void loop() {
  // Read and filter sensor data
  int leftDist = getFilteredDistance(sensorLeft, leftDistHistory);
  int rightDist = getFilteredDistance(sensorRight, rightDistHistory);
  int frontDist = getFilteredDistance(sensorFront, frontDistHistory);
  filterIndex++;

  readMPU6050();
  updateHeading();
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;

  float error = 0.0;

  if (frontDist < frontStopDistance) {
    isTurning = true;
    if (leftDist > rightDist) {
      leftSpeed = baseSpeed;
      rightSpeed = -baseSpeed;
      targetHeading = heading + 90.0;
    } else {
      leftSpeed = -baseSpeed;
      rightSpeed = baseSpeed;
      targetHeading = heading - 90.0;
    }
    integral = 0.0;
    previousError = 0.0;
  } else {
    // Check if turn is complete
    float headingError = abs(targetHeading - heading);
    if (isTurning && headingError < 5.0) { // Within 5° of target
      isTurning = false;
      targetHeading = heading; // Reset to current heading
    }

    if (!isTurning) {
      bool leftDetected = leftDist < desiredDistance * 2;
      bool rightDetected = rightDist < desiredDistance * 2;
      
      // Hysteresis for wall selection
      if (leftDetected && rightDetected) {
        if (followingLeft && rightDist < leftDist - HYSTERESIS_THRESHOLD) {
          followingLeft = false;
        } else if (!followingLeft && leftDist < rightDist - HYSTERESIS_THRESHOLD) {
          followingLeft = true;
        }
      } else if (leftDetected) {
        followingLeft = true;
      } else if (rightDetected) {
        followingLeft = false;
      } else {
        followingLeft = false; // Default to right if no walls
      }

      if (leftDetected || rightDetected) {
        error = (followingLeft ? leftDist : rightDist) - desiredDistance;
        if (abs(error) < 5) error = 0.0;
        
        integral += error * dt;
        integral = constrain(integral, -50.0, 50.0);
        float derivative = (error - previousError) / dt;
        float correction = Kp * error + Ki * integral + Kd * derivative;
        correction = constrain(correction, -15.0, 15.0); // Tighter limit
        previousError = error;

        if (followingLeft) {
          rightSpeed += (int)correction;
        } else {
          leftSpeed += (int)correction;
        }
      } else {
        integral = 0.0;
        previousError = 0.0;
        targetHeading = heading;
      }

      // Apply heading correction only when not turning
      applyHeadingCorrection(leftSpeed, rightSpeed);
    }
  }

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  if (millis() - lastBalanceTime > BALANCE_INTERVAL) {
    balanceMotors(leftDist, rightDist, frontDist);
    lastBalanceTime = millis();
  }

  int balancedRightSpeed = rightSpeed * motorBalance;
  
  digitalWrite(AIN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, leftSpeed > 0 ? LOW : HIGH);
  ledcWrite(0, abs(leftSpeed));

  digitalWrite(BIN1, balancedRightSpeed > 0 ? LOW : HIGH);
  digitalWrite(BIN2, balancedRightSpeed > 0 ? HIGH : LOW);
  ledcWrite(1, abs(balancedRightSpeed));

  Serial.printf("TOF: L=%3d F=%3d R=%3d | ", leftDist, frontDist, rightDist);
  Serial.printf("MOTOR: L=%3d R=%3d | ", leftSpeed, balancedRightSpeed);
  Serial.printf("ENC: L=%5ld R=%5ld | ", leftEncoderCount, rightEncoderCount);
  Serial.printf("ACC: X=%-5.1fg Y=%-5.1fg Z=%-5.1fg | ", accelX, accelY, accelZ);
  Serial.printf("GYRO: X=%-5.0f Y=%-5.0f Z=%-5.0f | ", gyroX, gyroY, gyroZ);
  Serial.printf("HEADING: %-5.1f TURN: %d\n", heading, isTurning);

  delay(50);
}

void readMPU6050() {
  if (mpu.testConnection()) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accelX = ax / 16384.0;
    accelY = ay / 16384.0; 
    accelZ = az / 16384.0;
    gyroX = gx / 131.0;
    gyroY = gy / 131.0;
    gyroZ = gz / 131.0;
  } else {
    Serial.println("MPU6050 read failed!");
    accelX = accelY = 0.0;
    accelZ = 1.0;
    gyroX = gyroY = gyroZ = 0.0;
  }
}

void balanceMotors(int leftDist, int rightDist, int frontDist) {
  static long lastLeft = 0, lastRight = 0;
  static int samples = 0;
  static float sumRatio = 0;
  
  if (leftDist >= 1000 && rightDist >= 1000 && frontDist >= 150 && !isTurning) {
    long leftDelta = leftEncoderCount - lastLeft;
    long rightDelta = rightEncoderCount - lastRight;
    
    if (leftDelta > MIN_DELTA_FOR_BALANCE && rightDelta > MIN_DELTA_FOR_BALANCE) {
      float instantRatio = (float)leftDelta / rightDelta;
      sumRatio += instantRatio;
      samples++;
      
      if (samples >= BALANCE_SAMPLES) {
        float avgRatio = sumRatio / samples;
        float correctionGain = 2.0; // Increased for stronger correction
        motorBalance *= avgRatio * correctionGain;
        motorBalance = constrain(motorBalance, 0.5f, 1.5f); // Wider range
        Serial.print("Motor balance adjusted to: ");
        Serial.println(motorBalance, 3);
        samples = 0;
        sumRatio = 0;
      }
    }
  }
  
  lastLeft = leftEncoderCount;
  lastRight = rightEncoderCount;
}

void updateHeading() {
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdate) / 1000.0;
  lastUpdate = currentTime;
  
  heading += gyroZ * dt;
  if (heading > 180.0) heading -= 360.0;
  if (heading < -180.0) heading += 360.0;
}

void applyHeadingCorrection(int &leftSpeed, int &rightSpeed) {
  if (!isTurning) {
    float headingError = targetHeading - heading;
    const float Kp_heading = 0.8; // Further reduced
    float correction = Kp_heading * headingError;
    correction = constrain(correction, -10.0, 10.0); // Tighter limit
    leftSpeed += (int)correction;
    rightSpeed -= (int)correction;
  }
}