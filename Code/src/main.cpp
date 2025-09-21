#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include <WiFi.h>

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

// Configuration
const int DESIRED_DISTANCE = 90;
const int BASE_SPEED = 80;
const int MAX_SPEED = 150;
const int FRONT_STOP_DISTANCE = 120;
const int FRONT_SLOW_DISTANCE = 180;
const int WALL_THRESHOLD = 200;

// PID Parameters
const float KP = 0.35;
const float KI = 0.002;
const float KD = 0.6;
const int DEADBAND = 5;

// Motion profiling parameters
const int MAX_SPEED_CHANGE = 3;
const float HEADING_CORRECTION_GAIN = 0.12;

// Turn parameters
const int TURN_SPEED = 60;  // Reduced from 100 for better control
const int TURN_ACCELERATION = 10;
const float TURN_ANGLE_TOLERANCE = 3.0;  // Tighter tolerance
const unsigned long TURN_TIMEOUT_MS = 5000;  // Longer timeout (renamed to avoid conflict)
const int POSITIONING_SPEED = 60;

// PID Variables
float previousError = 0.0;
float integral = 0.0;
const float MAX_INTEGRAL = 15.0;
unsigned long lastTime = 0;

// MPU6050 Variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
float heading = 0.0;
float targetHeading = 0.0;
bool headingInitialized = false;

// Sensor Smoothing
const int FILTER_SIZE = 7;
int leftDistHistory[FILTER_SIZE];
int rightDistHistory[FILTER_SIZE];
int frontDistHistory[FILTER_SIZE];

// Robot States
enum RobotState { FOLLOWING, TURNING, POSITIONING, STOPPED };
RobotState currentState = FOLLOWING;

// Wall following side
bool followingLeft = true;

// Turn variables
unsigned long turnStartTime = 0;
float turnStartHeading = 0.0;
float targetTurnAngle = 0.0;

// Positioning after turn
unsigned long positioningStartTime = 0;
const unsigned long POSITIONING_TIMEOUT = 1500;

// Motor smoothing
int currentLeftSpeed = 0;
int currentRightSpeed = 0;

// Low-pass filter for sensor readings
float filteredLeftDist = 0;
float filteredRightDist = 0;
float filteredFrontDist = 0;
const float FILTER_ALPHA = 0.3;

// WiFi Configuration
const char* ssid = "ABC";
const char* password = "zzom5037";
WiFiServer server(23);
WiFiClient client;

//accelerometer calibration offsets
float gyroZeroOffset = 0.0;
const float GYRO_DRIFT_ALPHA = 0.001;
unsigned long lastStillTime = 0;
const unsigned long STILL_TIME_THRESHOLD = 2000;

// Function prototypes
void handleCorner(float leftDist, float rightDist, float frontDist);
void followLeftWall(int &leftSpeed, int &rightSpeed, float leftDist, float dt);
void followRightWall(int &leftSpeed, int &rightSpeed, float rightDist, float dt);
bool isTurnComplete();
bool isPositioningComplete();
void handleWallFollowing(int &leftSpeed, int &rightSpeed, float leftDist, float rightDist, float frontDist, float dt);
void applyHeadingCorrection(int &leftSpeed, int &rightSpeed);
void setMotorsSmooth(int leftTarget, int rightTarget);
float getFilteredDistance(VL53L0X &sensor, float &filteredValue);
void readMPU6050();
void updateHeading();
void sendToClient(String message);
void applyTurnVelocityProfile(int &leftSpeed, int &rightSpeed, int targetLeft, int targetRight);
void stopMotors();
void debugCornerDetection(float leftDist, float rightDist, float frontDist);
int getOptimalTurnDirection(float currentHeading, float targetHeading);

void setup() {
  Serial.begin(115200);
  Wire.begin(15, 4);
  Wire.setClock(50000);

  // WiFi Connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP address: ");
  Serial.println(WiFi.localIP());
  
  // Start TCP server
  server.begin();
  Serial.println("TCP server started on port 23");

  // Initialize MPU6050
  mpu.initialize();
  delay(100);
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed!");
  }
  
  // Calibration
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
  sensorLeft.setMeasurementTimingBudget(20000);
  
  digitalWrite(xshutRight, HIGH);
  delay(15);
  if(!sensorRight.init()) { Serial.println("Right sensor failed!"); while(1); }
  sensorRight.setAddress(0x32);
  sensorRight.setTimeout(500);
  sensorRight.startContinuous();
  sensorRight.setMeasurementTimingBudget(20000);
  
  digitalWrite(xshutFront, HIGH);
  delay(15);
  if(!sensorFront.init()) { Serial.println("Front sensor failed!"); while(1); }
  sensorFront.setAddress(0x34);
  sensorFront.setTimeout(500);
  sensorFront.startContinuous();
  sensorFront.setMeasurementTimingBudget(20000);

  // Initialize sensor history
  for (int i = 0; i < FILTER_SIZE; i++) {
    leftDistHistory[i] = DESIRED_DISTANCE;
    rightDistHistory[i] = DESIRED_DISTANCE;
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

  lastTime = millis();
  Serial.println("System Ready - Improved Wall Following!");
}

void sendToClient(String message) {
  Serial.print(message);
  if (client && client.connected()) {
    client.print(message);
  }
}

void stopMotors() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  ledcWrite(0, 0);
  ledcWrite(1, 0);
}

float getFilteredDistance(VL53L0X &sensor, float &filteredValue) {
  int raw = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || raw > 1000) raw = 1000;
  
  filteredValue = FILTER_ALPHA * raw + (1 - FILTER_ALPHA) * filteredValue;
  return filteredValue;
}

void readMPU6050() {
  if (mpu.testConnection()) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }
}

void updateHeading() {
  static unsigned long lastUpdate = 0;
  static float gyroBias = 0.0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdate) / 1000.0;
  
  if (dt > 0 && dt < 0.1) {
    float gyroZ = (gz / 131.0) - gyroBias;
    heading += gyroZ * dt;
    
    // Auto-calibrate gyro bias when robot is still
    if (currentState == FOLLOWING && abs(gyroZ) < 1.0) {
      if (currentTime - lastStillTime > STILL_TIME_THRESHOLD) {
        gyroBias = gyroBias * 0.99 + (gz / 131.0) * 0.01;
      }
    } else {
      lastStillTime = currentTime;
    }
    
    // Keep heading in [-180, 180] range
    if (heading > 180.0) heading -= 360.0;
    if (heading < -180.0) heading += 360.0;
  }
  
  lastUpdate = currentTime;
  
  if (!headingInitialized) {
    targetHeading = heading;
    headingInitialized = true;
  }
}

void setMotorsSmooth(int leftTarget, int rightTarget) {
  if (leftTarget > currentLeftSpeed) {
    currentLeftSpeed = min(currentLeftSpeed + MAX_SPEED_CHANGE, leftTarget);
  } else if (leftTarget < currentLeftSpeed) {
    currentLeftSpeed = max(currentLeftSpeed - MAX_SPEED_CHANGE, leftTarget);
  }
  
  if (rightTarget > currentRightSpeed) {
    currentRightSpeed = min(currentRightSpeed + MAX_SPEED_CHANGE, rightTarget);
  } else if (rightTarget < currentRightSpeed) {
    currentRightSpeed = max(currentRightSpeed - MAX_SPEED_CHANGE, rightTarget);
  }
  
  digitalWrite(AIN1, currentLeftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, currentLeftSpeed > 0 ? LOW : HIGH);
  ledcWrite(0, abs(currentLeftSpeed));

  digitalWrite(BIN1, currentRightSpeed > 0 ? LOW : HIGH);
  digitalWrite(BIN2, currentRightSpeed > 0 ? HIGH : LOW);
  ledcWrite(1, abs(currentRightSpeed));
}

void applyTurnVelocityProfile(int &leftSpeed, int &rightSpeed, int targetLeft, int targetRight) {
  static int currentLeft = 0;
  static int currentRight = 0;
  
  // Gradually approach target speeds
  if (currentLeft < targetLeft) {
    currentLeft = min(currentLeft + TURN_ACCELERATION, targetLeft);
  } else if (currentLeft > targetLeft) {
    currentLeft = max(currentLeft - TURN_ACCELERATION, targetLeft);
  }
  
  if (currentRight < targetRight) {
    currentRight = min(currentRight + TURN_ACCELERATION, targetRight);
  } else if (currentRight > targetRight) {
    currentRight = max(currentRight - TURN_ACCELERATION, targetRight);
  }
  
  leftSpeed = currentLeft;
  rightSpeed = currentRight;
}

void applyHeadingCorrection(int &leftSpeed, int &rightSpeed) {
  if (currentState != FOLLOWING) return;
  
  float headingError = targetHeading - heading;
  
  while (headingError > 180.0) headingError -= 360.0;
  while (headingError < -180.0) headingError += 360.0;
  
  if (abs(headingError) < 1.5) return;
  
  float correction = headingError * HEADING_CORRECTION_GAIN;
  correction = constrain(correction, -4.0, 4.0);
  
  leftSpeed += (int)correction;
  rightSpeed -= (int)correction;
}

void handleWallFollowing(int &leftSpeed, int &rightSpeed, float leftDist, float rightDist, float frontDist, float dt) {
  // Emergency stop if too close to front wall
  if (frontDist < 80) {
    leftSpeed = 0;
    rightSpeed = 0;
    return;
  }
  
  if (leftDist < 50) {
    leftSpeed = BASE_SPEED + 25;
    rightSpeed = BASE_SPEED - 25;
    return;
  } else if (rightDist < 50) {
    leftSpeed = BASE_SPEED - 25;
    rightSpeed = BASE_SPEED + 25;
    return;
  }
  
  if (frontDist < FRONT_SLOW_DISTANCE) {
    float slowdownFactor = 1.0 - (0.6 * (FRONT_SLOW_DISTANCE - frontDist) / (FRONT_SLOW_DISTANCE - FRONT_STOP_DISTANCE));
    slowdownFactor = constrain(slowdownFactor, 0.4, 1.0);
    leftSpeed *= slowdownFactor;
    rightSpeed *= slowdownFactor;
  }
  
  if (frontDist < FRONT_STOP_DISTANCE) {
    handleCorner(leftDist, rightDist, frontDist);
    leftSpeed = 0;
    rightSpeed = 0;
    return;
  }
  
  bool leftWallValid = leftDist < WALL_THRESHOLD;
  bool rightWallValid = rightDist < WALL_THRESHOLD;
  
  if (leftWallValid && !rightWallValid) {
    followingLeft = true;
    targetHeading = heading;
  } else if (rightWallValid && !leftWallValid) {
    followingLeft = false;
    targetHeading = heading;
  }
  
  if (followingLeft) {
    followLeftWall(leftSpeed, rightSpeed, leftDist, dt);
  } else {
    followRightWall(leftSpeed, rightSpeed, rightDist, dt);
  }
}

void followLeftWall(int &leftSpeed, int &rightSpeed, float leftDist, float dt) {
  float error = leftDist - DESIRED_DISTANCE;
  
  float adaptiveKp = KP;
  float adaptiveKd = KD;
  
  if (leftDist < 70) {
    adaptiveKp = KP * 1.8;
    adaptiveKd = KD * 1.5;
  } else if (leftDist < 100) {
    adaptiveKp = KP * 1.3;
    adaptiveKd = KD * 1.2;
  }
  
  if (abs(error) < DEADBAND) {
    error = 0;
    integral *= 0.7;
  }
  
  integral += error * dt;
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  float derivative = (error - previousError) / dt;
  
  float correction = adaptiveKp * error + KI * integral + adaptiveKd * derivative;
  correction = constrain(correction, -25.0, 25.0);
  
  rightSpeed += (int)correction;
  previousError = error;
}

void followRightWall(int &leftSpeed, int &rightSpeed, float rightDist, float dt) {
  float error = rightDist - DESIRED_DISTANCE;
  
  float adaptiveKp = KP;
  float adaptiveKd = KD;
  
  if (rightDist < 70) {
    adaptiveKp = KP * 1.8;
    adaptiveKd = KD * 1.5;
  } else if (rightDist < 100) {
    adaptiveKp = KP * 1.3;
    adaptiveKd = KD * 1.2;
  }
  
  if (abs(error) < DEADBAND) {
    error = 0;
    integral *= 0.7;
  }
  
  integral += error * dt;
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  float derivative = (error - previousError) / dt;
  
  float correction = adaptiveKp * error + KI * integral + adaptiveKd * derivative;
  correction = constrain(correction, -25.0, 25.0);
  
  leftSpeed += (int)correction;
  previousError = error;
}

// REPLACE the handleCorner function with this:
void handleCorner(float leftDist, float rightDist, float frontDist) {
  // More conservative corner detection
  bool leftOpen = leftDist > WALL_THRESHOLD * 2.0;  // Even more sensitive
  bool rightOpen = rightDist > WALL_THRESHOLD * 2.0; // Even more sensitive
  bool frontVeryClose = frontDist < FRONT_STOP_DISTANCE * 0.7; // Even more sensitive
  
  // Only trigger if we have a clear corner (one side definitely open)
  if (!frontVeryClose || (!leftOpen && !rightOpen)) {
    return; // Not a clear corner
  }
  
  Serial.print("CLEAR CORNER DETECTED: ");
  Serial.printf("L=%.1f R=%.1f F=%.1f | ", leftDist, rightDist, frontDist);
  
  // Calculate target heading
  targetTurnAngle = 0;
  
  if (rightOpen && !leftOpen) {
    Serial.println("TURNING RIGHT (-90°)");
    targetTurnAngle = -90.0;
    followingLeft = false;
  } else if (leftOpen && !rightOpen) {
    Serial.println("TURNING LEFT (+90°)");
    targetTurnAngle = 90.0;
    followingLeft = true;
  } else if (rightOpen && leftOpen) {
    if (followingLeft) {
      Serial.println("TURNING LEFT (+90°, T-junction)");
      targetTurnAngle = 90.0;
    } else {
      Serial.println("TURNING RIGHT (-90°, T-junction)");
      targetTurnAngle = -90.0;
    }
  } else {
    Serial.println("DEAD END - U-TURN (+180°)");
    targetTurnAngle = 180.0;
    followingLeft = !followingLeft;
  }
  
  currentState = TURNING;
  turnStartTime = millis();
  turnStartHeading = heading;
  
  // Calculate target heading
  targetHeading = heading + targetTurnAngle;
  
  // Normalize to [-180, 180]
  if (targetHeading > 180.0) targetHeading -= 360.0;
  if (targetHeading < -180.0) targetHeading += 360.0;
  
  integral = 0;
  previousError = 0;
  
  Serial.printf("Start: %.1f° -> Target: %.1f° (Turn: %.1f°)\n", 
                heading, targetHeading, targetTurnAngle);
                
  // Immediately stop the robot
  stopMotors();
}

bool isTurnComplete() {
  float headingError = targetHeading - heading;
  
  // Normalize error to [-180, 180]
  while (headingError > 180.0) headingError -= 360.0;
  while (headingError < -180.0) headingError += 360.0;
  
  // Calculate the actual angle turned
  float turnedAngle = heading - turnStartHeading;
  while (turnedAngle > 180.0) turnedAngle -= 360.0;
  while (turnedAngle < -180.0) turnedAngle += 360.0;
  
  // Debug output
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 300) {
    Serial.printf("TURN: Cur=%.1f Start=%.1f Targ=%.1f Err=%.1f Turned=%.1f\n", 
                 heading, turnStartHeading, targetHeading, headingError, turnedAngle);
    lastDebug = millis();
  }
  
  // Completion conditions
  bool closeToTarget = fabs(headingError) < TURN_ANGLE_TOLERANCE;
  bool turnedEnough = fabs(turnedAngle) >= fabs(targetTurnAngle) * 0.8;
  bool timeout = (millis() - turnStartTime > TURN_TIMEOUT_MS);
  
  return (closeToTarget && turnedEnough) || timeout;
}

bool isPositioningComplete() {
  return (millis() - positioningStartTime > POSITIONING_TIMEOUT);
}

void debugCornerDetection(float leftDist, float rightDist, float frontDist) {
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {
    bool leftOpen = leftDist > WALL_THRESHOLD;
    bool rightOpen = rightDist > WALL_THRESHOLD;
    bool frontClose = frontDist < FRONT_STOP_DISTANCE;
    
    Serial.printf("CORNER DEBUG: L=%d R=%d F=%d | L_Open=%d R_Open=%d F_Close=%d\n", 
                 (int)leftDist, (int)rightDist, (int)frontDist, 
                 leftOpen, rightOpen, frontClose);
    lastDebug = millis();
  }
}

int getOptimalTurnDirection(float currentHeading, float targetHeading) {
  float error = targetHeading - currentHeading;
  
  // Normalize to [-180, 180]
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;
  
  return (error > 0) ? 1 : -1; // 1 = left, -1 = right
}

void loop() {
  // Handle TCP client connections
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("New client connected");
      client.println("Robot Controller Connected");
    }
  }
  
  // Read sensors
  float leftDist = getFilteredDistance(sensorLeft, filteredLeftDist);
  float rightDist = getFilteredDistance(sensorRight, filteredRightDist);
  float frontDist = getFilteredDistance(sensorFront, filteredFrontDist);
  
  // Debug corner detection
  debugCornerDetection(leftDist, rightDist, frontDist);

  // Update IMU
  readMPU6050();
  updateHeading();
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt > 0.1) dt = 0.01;
  lastTime = currentTime;
  
  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;
  
  // State machine
  switch (currentState) {
    case FOLLOWING:
      handleWallFollowing(leftSpeed, rightSpeed, leftDist, rightDist, frontDist, dt);
      applyHeadingCorrection(leftSpeed, rightSpeed);
      break;
      
// REPLACE the TURNING case with this improved version:
// REPLACE the TURNING case with this simplified version:
case TURNING: {
  float headingError = targetHeading - heading;
  
  // Normalize error to [-180, 180]
  while (headingError > 180.0) headingError -= 360.0;
  while (headingError < -180.0) headingError += 360.0;
  
  // Debug output
  static unsigned long lastTurnDebug = 0;
  if (millis() - lastTurnDebug > 200) {
    Serial.printf("TURNING: Cur=%.1f° Targ=%.1f° Err=%.1f°\n", heading, targetHeading, headingError);
    lastTurnDebug = millis();
  }
  
  // Calculate turn speed based on error (PROPORTIONAL CONTROL)
  int turnSpeed = constrain(fabs(headingError) * 1.5, 30, TURN_SPEED);
  
  // Determine turn direction
  int targetLeftSpeed, targetRightSpeed;
  
  if (headingError > 0) {
    // Need to turn LEFT (counter-clockwise)
    targetLeftSpeed = -turnSpeed;
    targetRightSpeed = turnSpeed;
  } else {
    // Need to turn RIGHT (clockwise)  
    targetLeftSpeed = turnSpeed;
    targetRightSpeed = -turnSpeed;
  }
  
  // Apply the turn speeds
  digitalWrite(AIN1, targetLeftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, targetLeftSpeed > 0 ? LOW : HIGH);
  ledcWrite(0, abs(targetLeftSpeed));

  digitalWrite(BIN1, targetRightSpeed > 0 ? LOW : HIGH);
  digitalWrite(BIN2, targetRightSpeed > 0 ? HIGH : LOW);
  ledcWrite(1, abs(targetRightSpeed));
  
  // Update current speeds for monitoring
  currentLeftSpeed = targetLeftSpeed;
  currentRightSpeed = targetRightSpeed;
  
  // Check if turn is complete - use a bit wider tolerance
  if (fabs(headingError) < 10.0 || millis() - turnStartTime > TURN_TIMEOUT_MS) {
    currentState = POSITIONING;
    positioningStartTime = millis();
    
    // Stop motors
    stopMotors();
    
    if (fabs(headingError) < 10.0) {
      Serial.printf("TURN COMPLETE: %.1f° -> %.1f° (Error: %.1f°)\n", 
                   turnStartHeading, heading, headingError);
    } else {
      Serial.println("TURN TIMEOUT, FORCING POSITIONING");
    }
  }
  break;
}
    
    case POSITIONING: {
      // Move forward to find the wall
      leftSpeed = POSITIONING_SPEED;
      rightSpeed = POSITIONING_SPEED;
      
      // Check if we found the target wall
      bool wallFound = false;
      if (followingLeft) {
        wallFound = (leftDist < WALL_THRESHOLD) && (leftDist > 30);
      } else {
        wallFound = (rightDist < WALL_THRESHOLD) && (rightDist > 30);
      }
      
      if (wallFound || isPositioningComplete()) {
        currentState = FOLLOWING;
        integral = 0;
        previousError = 0;
        targetHeading = heading;
        if (wallFound) Serial.println("POSITIONING COMPLETE - WALL FOUND");
        else Serial.println("POSITIONING COMPLETE - TIMEOUT");
      }
      break;
    }
      
    case STOPPED:
      leftSpeed = 0;
      rightSpeed = 0;
      break;
  }
  
  setMotorsSmooth(leftSpeed, rightSpeed);
  
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 100) {
    String message = "L=" + String(leftDist) + " F=" + String(frontDist) + " R=" + String(rightDist) + " | ";
    message += "MOTOR: L=" + String(currentLeftSpeed) + " R=" + String(currentRightSpeed) + " | ";
    message += "HEAD: " + String(heading, 1) + " TARG: " + String(targetHeading, 1) + " | ";
    message += "STATE: ";
    
    switch(currentState) {
      case FOLLOWING: message += "FOLLOW"; break;
      case TURNING: message += "TURN"; break;
      case POSITIONING: message += "POSITION"; break;
      case STOPPED: message += "STOP"; break;
    }
    
    message += " | SIDE: " + String(followingLeft ? "LEFT" : "RIGHT");
    message += "\n";
    
    sendToClient(message);
    lastDebugTime = millis();
  }
  
  delay(20);
}