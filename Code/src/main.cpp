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
const int baseSpeed = 40;         
const int maxSpeed = 100;         
const int minSpeed = 40;          
const int frontStopDistance = 250;  // Increased from 150
const int frontSlowDistance = 350;

// PID Parameters
const float Kp = 0.15;
const float Ki = 0.01;
const float Kd = 2.0;
const int DEADBAND = 5;

// PID Variables
float previousError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;

// Motor Balancing Parameters
const float INITIAL_BALANCE = 0.9f;
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
unsigned long turnStartTime = 0;
const unsigned long MAX_TURN_TIME = 3000;

// Sensor Smoothing
const int FILTER_SIZE = 7;
int leftDistHistory[FILTER_SIZE];
int rightDistHistory[FILTER_SIZE];
int frontDistHistory[FILTER_SIZE];
int filterIndex = 0;

// Wall Selection Hysteresis
const int HYSTERESIS_THRESHOLD = 20;
bool followingLeft = false;

// Robot States
enum RobotState { FOLLOWING, TURNING, TURNING_90, REVERSING, STOPPED, RECOVERING };
RobotState currentState = FOLLOWING;

// Motor smoothing
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int MAX_SPEED_CHANGE = 3;

// Turn completion detection
const float TURN_COMPLETE_THRESHOLD = 15.0;
const int WALL_DETECTION_THRESHOLD = 500;

// Heading correction
const float HEADING_CORRECTION_DEADBAND = 3.0;
unsigned long lastHeadingUpdate = 0;

// Corner detection
const int CORNER_DETECTION_THRESHOLD = 300; // mm for side wall detection in corners
const int REVERSE_DISTANCE = 200; // mm to reverse before turning

// Function prototypes
void readMPU6050();
void updateHeading();
void applyHeadingCorrection(int &leftSpeed, int &rightSpeed);
void balanceMotors(int leftDist, int rightDist, int frontDist);
void setMotorsSmoothly(int targetLeft, int targetRight);
int getMedianDistance(VL53L0X &sensor, int *history);
bool isTurnComplete();
void handleCorner(int &leftSpeed, int &rightSpeed, int leftDist, int rightDist, int frontDist);
void handleFollowingState(int &leftSpeed, int &rightSpeed, int leftDist, int rightDist, int frontDist, float dt);
void handleTurningState(int &leftSpeed, int &rightSpeed);
void handleReversingState(int &leftSpeed, int &rightSpeed);

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
  targetHeading = 0.0;
  Serial.println("System Ready with Improved Corner Handling!");
}

int getMedianDistance(VL53L0X &sensor, int *history) {
  int raw = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || raw > 1000) raw = 1000;
  
  for (int i = FILTER_SIZE-1; i > 0; i--) {
    history[i] = history[i-1];
  }
  history[0] = raw;
  
  int temp[FILTER_SIZE];
  memcpy(temp, history, sizeof(temp));
  
  for (int i = 0; i < FILTER_SIZE-1; i++) {
    for (int j = i+1; j < FILTER_SIZE; j++) {
      if (temp[j] < temp[i]) {
        int swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }
  
  return temp[FILTER_SIZE/2];
}

void loop() {
  int leftDist = getMedianDistance(sensorLeft, leftDistHistory);
  int rightDist = getMedianDistance(sensorRight, rightDistHistory);
  int frontDist = getMedianDistance(sensorFront, frontDistHistory);

  readMPU6050();
  updateHeading();
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  int targetLeftSpeed = baseSpeed;
  int targetRightSpeed = baseSpeed;

  // State machine
  switch (currentState) {
    case FOLLOWING:
      handleFollowingState(targetLeftSpeed, targetRightSpeed, leftDist, rightDist, frontDist, dt);
      break;
    case TURNING:
      handleTurningState(targetLeftSpeed, targetRightSpeed);
      break;
    case TURNING_90:
      handleTurningState(targetLeftSpeed, targetRightSpeed);
      break;
    case REVERSING:
      handleReversingState(targetLeftSpeed, targetRightSpeed);
      break;
    case RECOVERING:
      // Gentle forward movement while looking for walls
      targetHeading = heading;
      applyHeadingCorrection(targetLeftSpeed, targetRightSpeed);
      
      if (leftDist < desiredDistance * 2 || rightDist < desiredDistance * 2) {
        currentState = FOLLOWING;
      }
      break;
    case STOPPED:
      targetLeftSpeed = 0;
      targetRightSpeed = 0;
      break;
  }

  targetLeftSpeed = constrain(targetLeftSpeed, -maxSpeed, maxSpeed);
  targetRightSpeed = constrain(targetRightSpeed, -maxSpeed, maxSpeed);

  if (millis() - lastBalanceTime > BALANCE_INTERVAL) {
    balanceMotors(leftDist, rightDist, frontDist);
    lastBalanceTime = millis();
  }

  int balancedRightSpeed = targetRightSpeed * motorBalance;
  
  setMotorsSmoothly(targetLeftSpeed, balancedRightSpeed);

  Serial.printf("TOF: L=%3d F=%3d R=%3d | ", leftDist, frontDist, rightDist);
  Serial.printf("MOTOR: L=%3d R=%3d | ", currentLeftSpeed, (int)(currentRightSpeed / motorBalance));
  Serial.printf("HEADING: %-5.1f TARGET: %-5.1f | ", heading, targetHeading);
  Serial.printf("STATE: ");
  
  switch(currentState) {
    case FOLLOWING: Serial.print("FOLLOWING"); break;
    case TURNING: Serial.print("TURNING"); break;
    case TURNING_90: Serial.print("TURNING_90"); break;
    case REVERSING: Serial.print("REVERSING"); break;
    case RECOVERING: Serial.print("RECOVERING"); break;
    case STOPPED: Serial.print("STOPPED"); break;
  }
  Serial.println();

  delay(50);
}

void handleFollowingState(int &leftSpeed, int &rightSpeed, int leftDist, int rightDist, int frontDist, float dt) {
  // Slow down when approaching an obstacle
  if (frontDist < frontSlowDistance && frontDist > frontStopDistance) {
    float slowdownFactor = map(frontDist, frontStopDistance, frontSlowDistance, 0.5, 1.0);
    leftSpeed *= slowdownFactor;
    rightSpeed *= slowdownFactor;
    Serial.printf("Slowing down: F=%d factor=%.1f | ", frontDist, slowdownFactor);
  }

  // Check for front obstacle and handle corner detection
  if (frontDist < frontStopDistance) {
    Serial.printf("Front wall detected: F=%d | ", frontDist);
    handleCorner(leftSpeed, rightSpeed, leftDist, rightDist, frontDist);
    return;
  }

  // Normal wall following
  bool leftDetected = leftDist < desiredDistance * 2;
  bool rightDetected = rightDist < desiredDistance * 2;
  
  // CONTINUOUSLY update target heading during wall following to prevent drift
  static unsigned long lastHeadingUpdate = 0;
  if (millis() - lastHeadingUpdate > 500) { // Update target heading every 500ms
    targetHeading = heading;
    lastHeadingUpdate = millis();
  }
  
  if (leftDetected && rightDetected) {
    if (followingLeft && rightDist < leftDist - HYSTERESIS_THRESHOLD) {
      followingLeft = false;
      targetHeading = heading; // Reset target when switching walls
    } else if (!followingLeft && leftDist < rightDist - HYSTERESIS_THRESHOLD) {
      followingLeft = true;
      targetHeading = heading; // Reset target when switching walls
    }
  } else if (leftDetected) {
    if (!followingLeft) {
      followingLeft = true;
      targetHeading = heading;
    }
  } else if (rightDetected) {
    if (followingLeft) {
      followingLeft = false;
      targetHeading = heading;
    }
  } else {
    followingLeft = false;
    currentState = RECOVERING;
    targetHeading = heading;
  }

  if (leftDetected || rightDetected) {
    float error = (followingLeft ? leftDist : rightDist) - desiredDistance;
    
    if (abs(error) < DEADBAND) {
      error = 0.0;
      integral = 0.0;
    }
    
    integral += error * dt;
    integral = constrain(integral, -30.0, 30.0);
    float derivative = (error - previousError) / dt;
    float correction = Kp * error + Ki * integral + Kd * derivative;
    correction = constrain(correction, -8.0, 8.0);
    previousError = error;

    if (followingLeft) {
      rightSpeed += (int)correction;
    } else {
      leftSpeed += (int)correction;
    }
  } else {
    integral = 0.0;
    previousError = 0.0;
  }

  // Apply stronger heading correction during wall following
  applyHeadingCorrection(leftSpeed, rightSpeed);
}

void handleCorner(int &leftSpeed, int &rightSpeed, int leftDist, int rightDist, int frontDist) {
  // Make corner detection more sensitive - lower the threshold
  bool leftWallClose = leftDist < 400;  // Increased sensitivity
  bool rightWallClose = rightDist < 400; // Increased sensitivity
  
  // Check if front wall is very close
  bool frontWallVeryClose = frontDist < 250; // Reduced from 300
  
  Serial.printf("Corner detection: L=%d R=%d F=%d | ", leftWallClose, rightWallClose, frontWallVeryClose);
  
  if ((leftWallClose && rightWallClose) || (frontDist < frontStopDistance)) {
    // Dead end or front wall - reverse and then turn
    currentState = REVERSING;
    leftSpeed = -baseSpeed;
    rightSpeed = -baseSpeed;
    Serial.println("Dead end detected - reversing");
  }
  else if (frontWallVeryClose) {
    // Front wall detected - initiate turn
    if (leftWallClose && !rightWallClose) {
      // Right turn (left wall is close, right is open)
      currentState = TURNING_90;
      isTurning = true;
      turnStartTime = millis();
      leftSpeed = baseSpeed;
      rightSpeed = -baseSpeed;
      targetHeading = heading - 90.0;
      if (targetHeading < -180.0) targetHeading += 360.0;
      Serial.println("Right turn detected");
    }
    else if (rightWallClose && !leftWallClose) {
      // Left turn (right wall is close, left is open)
      currentState = TURNING_90;
      isTurning = true;
      turnStartTime = millis();
      leftSpeed = -baseSpeed;
      rightSpeed = baseSpeed;
      targetHeading = heading + 90.0;
      if (targetHeading > 180.0) targetHeading -= 360.0;
      Serial.println("Left turn detected");
    }
    else {
      // No clear side wall - turn based on current following direction
      currentState = TURNING;
      isTurning = true;
      turnStartTime = millis();
      
      if (followingLeft) {
        leftSpeed = baseSpeed;
        rightSpeed = -baseSpeed;
        targetHeading = heading + 90.0;
      } else {
        leftSpeed = -baseSpeed;
        rightSpeed = baseSpeed;
        targetHeading = heading - 90.0;
      }
      Serial.println("Front wall - turning based on current direction");
    }
  }
  
  integral = 0.0;
  previousError = 0.0;
}

void handleTurningState(int &leftSpeed, int &rightSpeed) {
  // Check if turn is complete using multiple criteria
  if (isTurnComplete()) {
    isTurning = false;
    currentState = FOLLOWING;
    targetHeading = heading;
    Serial.println("Turn completed");
    return;
  }
  
  // Timeout safety
  if (millis() - turnStartTime > MAX_TURN_TIME) {
    isTurning = false;
    currentState = RECOVERING;
    targetHeading = heading;
    Serial.println("Turn timeout - switching to recovery");
  }
}

void handleReversingState(int &leftSpeed, int &rightSpeed) {
  static unsigned long reverseStartTime = 0;
  static bool reverseStarted = false;
  
  if (!reverseStarted) {
    reverseStartTime = millis();
    reverseStarted = true;
    leftSpeed = -baseSpeed;
    rightSpeed = -baseSpeed;
    Serial.println("Starting reverse maneuver");
  }
  
  // Reverse for a short distance/time
  if (millis() - reverseStartTime > 1000) { // Reverse for 1 second
    reverseStarted = false;
    currentState = TURNING;
    isTurning = true;
    turnStartTime = millis();
    
    // Turn 180 degrees to face away from dead end
    leftSpeed = baseSpeed;
    rightSpeed = -baseSpeed;
    targetHeading = heading + 180.0;
    if (targetHeading > 180.0) targetHeading -= 360.0;
    Serial.println("Reverse completed - turning 180 degrees");
  }
}

bool isTurnComplete() {
  // Check heading-based completion
  float headingError = abs(targetHeading - heading);
  if (headingError > 180.0) headingError = 360.0 - headingError;
  
  if (headingError < TURN_COMPLETE_THRESHOLD) {
    return true;
  }
  
  // Check if we see a wall in front (indicating we completed the turn)
  int frontDist = getMedianDistance(sensorFront, frontDistHistory);
  if (frontDist < WALL_DETECTION_THRESHOLD && frontDist > 50) {
    return true;
  }
  
  return false;
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
        float correctionGain = 1.5;
        motorBalance *= avgRatio * correctionGain;
        motorBalance = constrain(motorBalance, 0.7f, 1.3f);
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
  
  // Keep heading in [-180, 180] range
  if (heading > 180.0) heading -= 360.0;
  if (heading < -180.0) heading += 360.0;
}

void applyHeadingCorrection(int &leftSpeed, int &rightSpeed) {
  if (currentState == RECOVERING || currentState == STOPPED) {
    // Full correction in recovery/stopped states
    float headingError = targetHeading - heading;
    
    if (headingError > 180.0) headingError -= 360.0;
    if (headingError < -180.0) headingError += 360.0;
    
    if (abs(headingError) < HEADING_CORRECTION_DEADBAND) {
      return;
    }
    
    const float Kp_heading = 0.3; // Increased gain
    float correction = Kp_heading * headingError;
    correction = constrain(correction, -6.0, 6.0); // Increased limits
    
    leftSpeed += (int)correction;
    rightSpeed -= (int)correction;
  }
  else if (currentState == FOLLOWING) {
    // Stronger heading correction during wall following
    float headingError = targetHeading - heading;
    
    if (headingError > 180.0) headingError -= 360.0;
    if (headingError < -180.0) headingError += 360.0;
    
    if (abs(headingError) > 5.0) {  // Correct if error > 5 degrees
      const float Kp_heading_wall = 0.15;  // Stronger correction during wall following
      float correction = Kp_heading_wall * headingError;
      correction = constrain(correction, -4.0, 4.0);
      
      leftSpeed += (int)correction;
      rightSpeed -= (int)correction;
    }
  }
}

void setMotorsSmoothly(int targetLeft, int targetRight) {
  if (targetLeft > currentLeftSpeed) {
    currentLeftSpeed = min(currentLeftSpeed + MAX_SPEED_CHANGE, targetLeft);
  } else if (targetLeft < currentLeftSpeed) {
    currentLeftSpeed = max(currentLeftSpeed - MAX_SPEED_CHANGE, targetLeft);
  }
  
  if (targetRight > currentRightSpeed) {
    currentRightSpeed = min(currentRightSpeed + MAX_SPEED_CHANGE, targetRight);
  } else if (targetRight < currentRightSpeed) {
    currentRightSpeed = max(currentRightSpeed - MAX_SPEED_CHANGE, targetRight);
  }
  
  digitalWrite(AIN1, currentLeftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, currentLeftSpeed > 0 ? LOW : HIGH);
  ledcWrite(0, abs(currentLeftSpeed));

  digitalWrite(BIN1, currentRightSpeed > 0 ? LOW : HIGH);
  digitalWrite(BIN2, currentRightSpeed > 0 ? HIGH : LOW);
  ledcWrite(1, abs(currentRightSpeed));
}