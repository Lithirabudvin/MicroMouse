#include <Wire.h>
#include "config.h"
#include "MotorControl.h"
#include "SensorManager.h"
#include "IMUHandler.h"
#include "WallFollower.h"
#include "NetworkManager.h"

// Global instances
MotorControl motors;
SensorManager sensors;
IMUHandler imu;
WallFollower wallFollower;
NetworkManager network;

unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_SPEED);

    // Initialize all systems
    Serial.println("=== WALL FOLLOWING ROBOT INITIALIZATION ===");
    
    // Network setup
    if (!network.setup()) {
        Serial.println("Network setup failed!");
        while(1);
    }
    
    // Sensor setup
    if (!sensors.setup()) {
        Serial.println("Sensor setup failed!");
        while(1);
    }
    
    // Motor setup
    motors.setup();
    
    // Initialize IMU heading
    imu.initializeHeading();
    
    // Initialize wall follower
    wallFollower.resetPID();
    
    lastTime = millis();
    Serial.println("=== SYSTEM READY - IMPROVED WALL FOLLOWING! ===");
    Serial.println();
}

void loop() {
    // Handle network connections
    network.handleClient();
    
    // Read sensors
    float leftDist, rightDist, frontDist;
    sensors.readSensors(leftDist, rightDist, frontDist);
    
    // Read IMU
    sensors.readMPU6050(imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz);
    imu.updateHeading(imu.gz, millis(), wallFollower.currentState);
    
    // Calculate timing
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    if (dt > MAX_DT) dt = MIN_DT;
    lastTime = currentTime;
    
    // Process robot behavior based on state
    int leftSpeed = WallFollower::getBaseSpeed();
    int rightSpeed = WallFollower::getBaseSpeed();
    
    // Store previous state to detect transitions
    static RobotState previousState = STOPPED;
    
    switch (wallFollower.currentState) {
        case FOLLOWING:
            wallFollower.handleWallFollowing(leftSpeed, rightSpeed, leftDist, rightDist, frontDist, dt, imu);
            wallFollower.applyHeadingCorrection(leftSpeed, rightSpeed, imu);
            break;
            
        case TURNING: {
            // Reset turn profile when first entering TURNING state
            if (previousState != TURNING) {
                wallFollower.resetTurnProfile();
                motors.resetTurnProfile();
                Serial.println("TURNING: Reset velocity profile");
            }
            
            float headingError = imu.getHeadingError(imu.targetHeading);
            
            #if DEBUG_TURN_PROGRESS
            static unsigned long lastTurnDebug = 0;
            if (millis() - lastTurnDebug > TURN_DEBUG_INTERVAL) {
                Serial.printf("TURNING: Cur=%.1f° Targ=%.1f° Err=%.1f°\n", imu.heading, imu.targetHeading, headingError);
                lastTurnDebug = millis();
            }
            #endif
            
            // Calculate turn speed based on error (PROPORTIONAL CONTROL)
            int turnSpeed = constrain(fabs(headingError) * 1.5, 30, WallFollower::getTurnSpeed());
            
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
            
            // APPLY VELOCITY PROFILING FOR SMOOTH TURN ACCELERATION
            int profiledLeftSpeed, profiledRightSpeed;
            motors.applyTurnVelocityProfile(profiledLeftSpeed, profiledRightSpeed, targetLeftSpeed, targetRightSpeed);
            
            // Apply profiled turn motors
            motors.setTurnMotors(profiledLeftSpeed, profiledRightSpeed);
            
            // Debug turn velocity profiling
            #if DEBUG_TURN_PROGRESS
            static unsigned long lastVelDebug = 0;
            if (millis() - lastVelDebug > 100) {
                Serial.printf("TURN VEL: Target(L=%d,R=%d) Profiled(L=%d,R=%d)\n", 
                             targetLeftSpeed, targetRightSpeed, profiledLeftSpeed, profiledRightSpeed);
                lastVelDebug = millis();
            }
            #endif
            
            // Check if turn is complete
            if (fabs(headingError) < TURN_TOLERANCE_WIDE || millis() - wallFollower.getTurnStartTime() > WallFollower::getTurnTimeout()) {
                wallFollower.currentState = POSITIONING;
                wallFollower.setPositioningStartTime(millis());
                
                // Stop motors
                motors.stopMotors();
                
                if (fabs(headingError) < TURN_TOLERANCE_WIDE) {
                    Serial.printf("TURN COMPLETE: %.1f° -> %.1f° (Error: %.1f°)\n", 
                                 wallFollower.getTurnStartHeading(), imu.heading, headingError);
                } else {
                    Serial.println("TURN TIMEOUT, FORCING POSITIONING");
                }
            }
            return; // Skip motor smoothing during turning
        }
        break;
        
        case POSITIONING: {
            // Move forward to find the wall
            leftSpeed = WallFollower::getPositioningSpeed();
            rightSpeed = WallFollower::getPositioningSpeed();
            
            // Check if we found the target wall
            bool wallFound = false;
            if (wallFollower.followingLeft) {
                wallFound = (leftDist < WallFollower::getWallThreshold()) && (leftDist > POSITIONING_MIN_DISTANCE);
            } else {
                wallFound = (rightDist < WallFollower::getWallThreshold()) && (rightDist > POSITIONING_MIN_DISTANCE);
            }
            
            if (wallFound || wallFollower.isPositioningComplete()) {
                wallFollower.currentState = FOLLOWING;
                wallFollower.resetPID();
                imu.targetHeading = imu.heading;
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
    
    // Apply motor control (skip during turning)
    if (wallFollower.currentState != TURNING) {
        motors.setMotorsSmooth(leftSpeed, rightSpeed);
    }
    
    // Update previous state for next iteration
    previousState = wallFollower.currentState;
    
    // Debug output
    #if DEBUG_SENSOR_READINGS
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > DEBUG_INTERVAL) {
        String message = "L=" + String(leftDist) + " F=" + String(frontDist) + " R=" + String(rightDist) + " | ";
        message += "MOTOR: L=" + String(motors.getCurrentLeftSpeed()) + " R=" + String(motors.getCurrentRightSpeed()) + " | ";
        message += "HEAD: " + String(imu.heading, 1) + " TARG: " + String(imu.targetHeading, 1) + " | ";
        message += "STATE: ";
        
        switch(wallFollower.currentState) {
            case FOLLOWING: message += "FOLLOW"; break;
            case TURNING: message += "TURN"; break;
            case POSITIONING: message += "POSITION"; break;
            case STOPPED: message += "STOP"; break;
        }
        
        message += " | SIDE: " + String(wallFollower.followingLeft ? "LEFT" : "RIGHT");
        message += "\n";
        
        network.sendMessage(message);
        lastDebugTime = millis();
    }
    #endif
    
    // Debug corner detection
    wallFollower.debugCornerDetection(leftDist, rightDist, frontDist);
    
    delay(LOOP_DELAY_MS);
}
