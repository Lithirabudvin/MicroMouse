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

// Wall-Following Parameters
const int DESIRED_DISTANCE = 90;      // Middle of 180mm corridor (90mm from each wall)
const int BASE_SPEED = 60;           
const int MAX_SPEED = 100;
const int FRONT_STOP_DISTANCE = 120;  // Reduced for 180mm corridor
const int FRONT_SLOW_DISTANCE = 180;  // Start slowing down earlier
const int WALL_THRESHOLD = 200;       // Threshold to consider a wall present

// Improved parameters for corner detection
const int SIDE_WALL_THRESHOLD = 70;   // mm - if wall is closer than this, we might be detecting corner
const int FRONT_CORNER_ANGLE_THRESHOLD = 30; // degrees - max angle difference for corner detection

// PID Parameters - more aggressive
const float KP = 0.5;                 // Increased from 0.4
const float KI = 0.001;               // Reduced to prevent integral windup
const float KD = 0.8;                 // Increased from 0.7
const int DEADBAND = 3;               // Reduced deadband for tighter control

// PID Variables
float previousError = 0.0;
float integral = 0.0;
const float MAX_INTEGRAL = 20.0;
unsigned long lastTime = 0;

// MPU6050 Variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
float heading = 0.0;
float targetHeading = 0.0;
bool headingInitialized = false;

// Sensor Smoothing
const int FILTER_SIZE = 5;
int leftDistHistory[FILTER_SIZE];
int rightDistHistory[FILTER_SIZE];
int frontDistHistory[FILTER_SIZE];

// Corner detection history
int prevLeftDist = 1000;
int prevRightDist = 1000;
int prevFrontDist = 1000;

// Robot States
enum RobotState { FOLLOWING, TURNING, REVERSING, STOPPED, POSITIONING };
RobotState currentState = FOLLOWING;

// Wall following side
bool followingLeft = true;

// Turn variables
unsigned long turnStartTime = 0;
const unsigned long TURN_TIMEOUT = 2000;
float turnStartHeading = 0.0;

// Positioning after turn
unsigned long positioningStartTime = 0;
const unsigned long POSITIONING_TIMEOUT = 500;

// Motor smoothing
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int MAX_SPEED_CHANGE = 5;

// Function prototypes
void handleCorner(int leftDist, int rightDist, int frontDist);
void followLeftWall(int &leftSpeed, int &rightSpeed, int leftDist, float dt);
void followRightWall(int &leftSpeed, int &rightSpeed, int rightDist, float dt);
bool isRealCorner(int leftDist, int rightDist, int frontDist);
bool isPositioningComplete();

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
  Serial.println("System Ready - Fixed Heading Control!");
}

int getMedianDistance(VL53L0X &sensor, int *history) {
  int raw = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || raw > 1000) raw = 1000;
  
  // Update history
  for (int i = FILTER_SIZE-1; i > 0; i--) {
    history[i] = history[i-1];
  }
  history[0] = raw;
  
  // Sort and return median
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

void readMPU6050() {
  if (mpu.testConnection()) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }
}

void updateHeading() {
  static unsigned long lastUpdate = 0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastUpdate) / 1000.0;
  
  if (dt > 0 && dt < 0.1) { // Prevent large dt values
    float gyroZ = gz / 131.0; // Convert to degrees/sec
    heading += gyroZ * dt;
    
    // Keep heading in [-180, 180] range
    if (heading > 180.0) heading -= 360.0;
    if (heading < -180.0) heading += 360.0;
  }
  
  lastUpdate = currentTime;
  
  // Initialize target heading on first update
  if (!headingInitialized) {
    targetHeading = heading;
    headingInitialized = true;
  }
}

void applyHeadingCorrection(int &leftSpeed, int &rightSpeed) {
  if (currentState != FOLLOWING) return;
  
  float headingError = targetHeading - heading;
  
  // Normalize error to [-180, 180]
  while (headingError > 180.0) headingError -= 360.0;
  while (headingError < -180.0) headingError += 360.0;
  
  if (abs(headingError) < 2.0) return; // Smaller deadband
  
  // Proportional correction
  float correction = headingError * 0.15; // Reduced gain for smoother correction
  correction = constrain(correction, -5.0, 5.0);
  
  leftSpeed += (int)correction;
  rightSpeed -= (int)correction;
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  // Smooth acceleration
  if (leftSpeed > currentLeftSpeed) {
    currentLeftSpeed = min(currentLeftSpeed + MAX_SPEED_CHANGE, leftSpeed);
  } else if (leftSpeed < currentLeftSpeed) {
    currentLeftSpeed = max(currentLeftSpeed - MAX_SPEED_CHANGE, leftSpeed);
  }
  
  if (rightSpeed > currentRightSpeed) {
    currentRightSpeed = min(currentRightSpeed + MAX_SPEED_CHANGE, rightSpeed);
  } else if (rightSpeed < currentRightSpeed) {
    currentRightSpeed = max(currentRightSpeed - MAX_SPEED_CHANGE, rightSpeed);
  }
  
  // Set motor directions and speeds
  digitalWrite(AIN1, currentLeftSpeed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, currentLeftSpeed > 0 ? LOW : HIGH);
  ledcWrite(0, abs(currentLeftSpeed));

  digitalWrite(BIN1, currentRightSpeed > 0 ? LOW : HIGH);
  digitalWrite(BIN2, currentRightSpeed > 0 ? HIGH : LOW);
  ledcWrite(1, abs(currentRightSpeed));
}

bool isRealCorner(int leftDist, int rightDist, int frontDist) {
  // A real corner typically has:
  // - Front distance decreasing rapidly
  // - One side distance increasing (the turn side)
  // - The other side distance decreasing (the wall side)
  
  bool isCorner = false;
  
  // Check if front distance is decreasing rapidly
  if (prevFrontDist - frontDist > 20 && frontDist < 150) {
    if (followingLeft) {
      // Following left wall, right turn should open up
      isCorner = (rightDist > prevRightDist + 20) && (rightDist > 300);
    } else {
      // Following right wall, left turn should open up
      isCorner = (leftDist > prevLeftDist + 20) && (leftDist > 300);
    }
  }
  
  prevLeftDist = leftDist;
  prevRightDist = rightDist;
  prevFrontDist = frontDist;
  
  return isCorner;
}

void handleWallFollowing(int &leftSpeed, int &rightSpeed, int leftDist, int rightDist, int frontDist, float dt) {
  // Emergency wall avoidance - highest priority
  if (leftDist < 50) {
    // Too close to left wall, move right urgently
    leftSpeed = BASE_SPEED + 30;
    rightSpeed = BASE_SPEED - 30;
    return;
  } else if (rightDist < 50) {
    // Too close to right wall, move left urgently
    leftSpeed = BASE_SPEED - 30;
    rightSpeed = BASE_SPEED + 30;
    return;
  }
  
  // Slow down for front obstacles
  if (frontDist < FRONT_SLOW_DISTANCE) {
    float slowdown = map(frontDist, FRONT_STOP_DISTANCE, FRONT_SLOW_DISTANCE, 0.4, 1.0);
    slowdown = constrain(slowdown, 0.4, 1.0);
    leftSpeed *= slowdown;
    rightSpeed *= slowdown;
  }
  
  // Check if we need to turn (front wall detected)
  bool tooCloseToLeft = leftDist < SIDE_WALL_THRESHOLD;
  bool tooCloseToRight = rightDist < SIDE_WALL_THRESHOLD;
  
  // Only check for front wall if we're not too close to a side wall
  if (frontDist < FRONT_STOP_DISTANCE && !tooCloseToLeft && !tooCloseToRight) {
    handleCorner(leftDist, rightDist, frontDist);
    return;
  }
  
  // If we're close to a side wall but front is also close, it might be a corner
  if (frontDist < FRONT_STOP_DISTANCE && (tooCloseToLeft || tooCloseToRight)) {
    // This might be a corner - check if it's a real corner or just wall proximity
    if (isRealCorner(leftDist, rightDist, frontDist)) {
      handleCorner(leftDist, rightDist, frontDist);
      return;
    } else {
      // Just too close to wall, adjust position instead of turning
      if (tooCloseToLeft) {
        rightSpeed -= 20; // Move away from left wall
      } else {
        leftSpeed -= 20; // Move away from right wall
      }
    }
  }
  
  // Determine which wall to follow based on which is closer
  bool leftWall = leftDist < WALL_THRESHOLD;
  bool rightWall = rightDist < WALL_THRESHOLD;
  
  if (leftWall && !rightWall) {
    followingLeft = true;
    targetHeading = heading; // Reset target when switching walls
  } else if (rightWall && !leftWall) {
    followingLeft = false;
    targetHeading = heading; // Reset target when switching walls
  }
  // If both walls detected, maintain current side
  
  if (followingLeft) {
    followLeftWall(leftSpeed, rightSpeed, leftDist, dt);
  } else {
    followRightWall(leftSpeed, rightSpeed, rightDist, dt);
  }
}

void followLeftWall(int &leftSpeed, int &rightSpeed, int leftDist, float dt) {
  float error = leftDist - DESIRED_DISTANCE;
  
  // Emergency correction when too close to wall
  if (leftDist < 60) {
    // Very close to wall - aggressive correction
    error = (leftDist - DESIRED_DISTANCE) * 2.5;
    integral = 0; // Reset integral to prevent windup
  } 
  // Aggressive correction when moderately close to wall
  else if (leftDist < 80) {
    error = (leftDist - DESIRED_DISTANCE) * 1.8;
  }
  // Moderate correction when getting close
  else if (leftDist < 100) {
    error = (leftDist - DESIRED_DISTANCE) * 1.3;
  }
  
  if (abs(error) < DEADBAND) {
    error = 0;
    integral *= 0.8; // Faster integral decay
  }
  
  integral += error * dt;
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  float derivative = (error - previousError) / dt;
  
  float correction = KP * error + KI * integral + KD * derivative;
  
  // More aggressive correction limits when close to walls
  if (leftDist < 80) {
    correction = constrain(correction, -30.0, 30.0);
  } else {
    correction = constrain(correction, -20.0, 20.0);
  }
  
  rightSpeed += (int)correction;
  previousError = error;
}

void followRightWall(int &leftSpeed, int &rightSpeed, int rightDist, float dt) {
  float error = rightDist - DESIRED_DISTANCE;
  
  // Emergency correction when too close to wall
  if (rightDist < 60) {
    // Very close to wall - aggressive correction
    error = (rightDist - DESIRED_DISTANCE) * 2.5;
    integral = 0; // Reset integral to prevent windup
  } 
  // Aggressive correction when moderately close to wall
  else if (rightDist < 80) {
    error = (rightDist - DESIRED_DISTANCE) * 1.8;
  }
  // Moderate correction when getting close
  else if (rightDist < 100) {
    error = (rightDist - DESIRED_DISTANCE) * 1.3;
  }
  
  if (abs(error) < DEADBAND) {
    error = 0;
    integral *= 0.8; // Faster integral decay
  }
  
  integral += error * dt;
  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
  float derivative = (error - previousError) / dt;
  
  float correction = KP * error + KI * integral + KD * derivative;
  
  // More aggressive correction limits when close to walls
  if (rightDist < 80) {
    correction = constrain(correction, -30.0, 30.0);
  } else {
    correction = constrain(correction, -20.0, 20.0);
  }
  
  leftSpeed += (int)correction;
  previousError = error;
}
void handleCorner(int leftDist, int rightDist, int frontDist) {
  Serial.print("CORNER DETECTED: ");
  
  // Check which sides are open - we want to turn toward the OPEN side
  bool leftOpen = leftDist > WALL_THRESHOLD;
  bool rightOpen = rightDist > WALL_THRESHOLD;
  
  Serial.printf("L=%s R=%s | ", leftOpen ? "OPEN" : "WALL", rightOpen ? "OPEN" : "WALL");
  
  if (rightOpen && !leftOpen) {
    // Turn right (right side is open)
    Serial.println("TURNING RIGHT");
    currentState = TURNING;
    turnStartTime = millis();
    turnStartHeading = heading;
    targetHeading = heading - 90.0;
    followingLeft = false;
    
  } else if (leftOpen && !rightOpen) {
    // Turn left (left side is open)
    Serial.println("TURNING LEFT");
    currentState = TURNING;
    turnStartTime = millis();
    turnStartHeading = heading;
    targetHeading = heading + 90.0;
    followingLeft = true;
    
  } else if (rightOpen && leftOpen) {
    // Both sides open - choose based on current following side
    if (followingLeft) {
      Serial.println("TURNING LEFT (both open)");
      currentState = TURNING;
      turnStartTime = millis();
      turnStartHeading = heading;
      targetHeading = heading + 90.0;
    } else {
      Serial.println("TURNING RIGHT (both open)");
      currentState = TURNING;
      turnStartTime = millis();
      turnStartHeading = heading;
      targetHeading = heading - 90.0;
    }
    
  } else {
    // Dead end - U-turn
    Serial.println("DEAD END - U-TURN");
    currentState = TURNING;
    turnStartTime = millis();
    turnStartHeading = heading;
    targetHeading = heading + 180.0;
    followingLeft = !followingLeft; // Switch wall side
  }
  
  // Normalize target heading
  if (targetHeading > 180.0) targetHeading -= 360.0;
  if (targetHeading < -180.0) targetHeading += 360.0;
  
  integral = 0;
  previousError = 0;
}

bool isTurnComplete() {
  float headingError = targetHeading - heading;
  
  // Normalize error to [-180, 180]
  while (headingError > 180.0) headingError -= 360.0;
  while (headingError < -180.0) headingError += 360.0;
  
  // Check if we've turned at least 80 degrees (allowing for some overshoot)
  float turnedAngle = abs(heading - turnStartHeading);
  if (turnedAngle > 180.0) turnedAngle = 360.0 - turnedAngle;
  
  return (turnedAngle >= 80.0) || (millis() - turnStartTime > TURN_TIMEOUT);
}

bool isPositioningComplete() {
  return (millis() - positioningStartTime > POSITIONING_TIMEOUT);
}

void loop() {
  // Read sensors
  int leftDist = getMedianDistance(sensorLeft, leftDistHistory);
  int rightDist = getMedianDistance(sensorRight, rightDistHistory);
  int frontDist = getMedianDistance(sensorFront, frontDistHistory);
  
  // Update IMU
  readMPU6050();
  updateHeading();
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt > 0.1) dt = 0.01; // Prevent large dt values
  lastTime = currentTime;
  
  int leftSpeed = BASE_SPEED;
  int rightSpeed = BASE_SPEED;
  
  // State machine
  switch (currentState) {
    case FOLLOWING:
      handleWallFollowing(leftSpeed, rightSpeed, leftDist, rightDist, frontDist, dt);
      applyHeadingCorrection(leftSpeed, rightSpeed);
      break;
      
    case TURNING: {
      // Smooth turning with proportional control
      float headingError = targetHeading - heading;
      
      // Normalize error to [-180, 180]
      while (headingError > 180.0) headingError -= 360.0;
      while (headingError < -180.0) headingError += 360.0;
      
      // Proportional turning control
      float turnRate = constrain(headingError * 0.8, -40.0, 40.0);
      
      if (headingError > 0) {
        // Need to turn right (clockwise)
        leftSpeed = BASE_SPEED/2 + turnRate;
        rightSpeed = BASE_SPEED/2 - turnRate;
      } else {
        // Need to turn left (counter-clockwise)
        leftSpeed = BASE_SPEED/2 - turnRate;
        rightSpeed = BASE_SPEED/2 + turnRate;
      }
      
      if (isTurnComplete()) {
        currentState = POSITIONING;
        positioningStartTime = millis();
        // Reset target heading to current heading after turn
        targetHeading = heading;
        Serial.println("TURN COMPLETE, POSITIONING");
      }
      break;
    }
    
case POSITIONING:
  // After a turn, position the robot away from the wall more aggressively
  if (followingLeft) {
    // Just turned to follow left wall, move right more aggressively
    leftSpeed = 50;
    rightSpeed = 20;
  } else {
    // Just turned to follow right wall, move left more aggressively  
    leftSpeed = 20;
    rightSpeed = 50;
  }
  
  if (isPositioningComplete()) {
    currentState = FOLLOWING;
    // Reset PID to prevent residual corrections from affecting new state
    integral = 0;
    previousError = 0;
    Serial.println("POSITIONING COMPLETE");
  }
  break;
      
    case REVERSING:
      leftSpeed = -BASE_SPEED/2;
      rightSpeed = -BASE_SPEED/2;
      break;
      
    case STOPPED:
      leftSpeed = 0;
      rightSpeed = 0;
      break;
  }
  
  setMotors(leftSpeed, rightSpeed);
  
  // Debug output
  Serial.printf("L=%3d F=%3d R=%3d | ", leftDist, frontDist, rightDist);
  Serial.printf("MOTOR: L=%3d R=%3d | ", currentLeftSpeed, currentRightSpeed);
  Serial.printf("HEAD: %6.1f TARG: %6.1f | ", heading, targetHeading);
  Serial.printf("STATE: ");
  
  switch(currentState) {
    case FOLLOWING: Serial.print("FOLLOW"); break;
    case TURNING: Serial.print("TURN"); break;
    case POSITIONING: Serial.print("POSITION"); break;
    case REVERSING: Serial.print("REVERSE"); break;
    case STOPPED: Serial.print("STOP"); break;
  }
  
  Serial.printf(" | SIDE: %s", followingLeft ? "LEFT" : "RIGHT");
  Serial.println();
  
  delay(50);
}