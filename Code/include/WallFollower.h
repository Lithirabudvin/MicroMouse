#ifndef WALLFOLLOWER_H
#define WALLFOLLOWER_H

#include "IMUHandler.h"
#include "config.h"

class WallFollower {
private:
    // PID Variables
    float previousError = 0.0;
    float integral = 0.0;
    
    // Turn variables (kept private, but provide accessors)
    unsigned long turnStartTime = 0;
    float turnStartHeading = 0.0;
    float targetTurnAngle = 0.0;
    unsigned long positioningStartTime = 0;

    void followLeftWall(int &leftSpeed, int &rightSpeed, float leftDist, float dt) {
        float error = leftDist - DESIRED_DISTANCE;
        
        float adaptiveKp = PID_KP;
        float adaptiveKd = PID_KD;
        
        if (leftDist < ADAPTIVE_DISTANCE_CLOSE) {
            adaptiveKp = PID_KP * ADAPTIVE_PID_CLOSE_FACTOR;
            adaptiveKd = PID_KD * ADAPTIVE_DERIVATIVE_CLOSE;
        } else if (leftDist < ADAPTIVE_DISTANCE_MEDIUM) {
            adaptiveKp = PID_KP * ADAPTIVE_PID_MEDIUM_FACTOR;
            adaptiveKd = PID_KD * ADAPTIVE_DERIVATIVE_MEDIUM;
        }
        
        if (abs(error) < PID_DEADBAND) {
            error = 0;
            integral *= 0.7;
        }
        
        integral += error * dt;
        integral = constrain(integral, -PID_MAX_INTEGRAL, PID_MAX_INTEGRAL);
        float derivative = (error - previousError) / dt;
        
        float correction = adaptiveKp * error + PID_KI * integral + adaptiveKd * derivative;
        correction = constrain(correction, -MAX_PID_CORRECTION, MAX_PID_CORRECTION);
        
        rightSpeed += (int)correction;
        previousError = error;
    }
    
    void followRightWall(int &leftSpeed, int &rightSpeed, float rightDist, float dt) {
        float error = rightDist - DESIRED_DISTANCE;
        
        float adaptiveKp = PID_KP;
        float adaptiveKd = PID_KD;
        
        if (rightDist < ADAPTIVE_DISTANCE_CLOSE) {
            adaptiveKp = PID_KP * ADAPTIVE_PID_CLOSE_FACTOR;
            adaptiveKd = PID_KD * ADAPTIVE_DERIVATIVE_CLOSE;
        } else if (rightDist < ADAPTIVE_DISTANCE_MEDIUM) {
            adaptiveKp = PID_KP * ADAPTIVE_PID_MEDIUM_FACTOR;
            adaptiveKd = PID_KD * ADAPTIVE_DERIVATIVE_MEDIUM;
        }
        
        if (abs(error) < PID_DEADBAND) {
            error = 0;
            integral *= 0.7;
        }
        
        integral += error * dt;
        integral = constrain(integral, -PID_MAX_INTEGRAL, PID_MAX_INTEGRAL);
        float derivative = (error - previousError) / dt;
        
        float correction = adaptiveKp * error + PID_KI * integral + adaptiveKd * derivative;
        correction = constrain(correction, -MAX_PID_CORRECTION, MAX_PID_CORRECTION);
        
        leftSpeed += (int)correction;
        previousError = error;
    }

public:
    bool followingLeft = true;
    RobotState currentState = FOLLOWING;
    
    // Accessors for constants used in main.cpp
    static int getBaseSpeed() { return BASE_SPEED; }
    static int getTurnSpeed() { return TURN_SPEED; }
    static int getWallThreshold() { return WALL_THRESHOLD; }
    static int getPositioningSpeed() { return POSITIONING_SPEED; }
    static unsigned long getTurnTimeout() { return TURN_TIMEOUT_MS; }
    
    // Accessors for turn variables (now non-static since they need instance data)
    unsigned long getTurnStartTime() { return turnStartTime; }
    float getTurnStartHeading() { return turnStartHeading; }
    void setPositioningStartTime(unsigned long time) { positioningStartTime = time; }
    
    void handleWallFollowing(int &leftSpeed, int &rightSpeed, float leftDist, float rightDist, float frontDist, float dt, IMUHandler &imu) {
        // Emergency stop if too close to front wall
        if (frontDist < EMERGENCY_STOP_DISTANCE) {
            leftSpeed = 0;
            rightSpeed = 0;
            return;
        }
        
        if (leftDist < CLOSE_WALL_DISTANCE) {
            leftSpeed = BASE_SPEED + 25;
            rightSpeed = BASE_SPEED - 25;
            return;
        } else if (rightDist < CLOSE_WALL_DISTANCE) {
            leftSpeed = BASE_SPEED - 25;
            rightSpeed = BASE_SPEED + 25;
            return;
        }
        
        if (frontDist < FRONT_SLOW_DISTANCE) {
            float slowdownFactor = 1.0 - (SLOWDOWN_FACTOR_MAX * (FRONT_SLOW_DISTANCE - frontDist) / (FRONT_SLOW_DISTANCE - FRONT_STOP_DISTANCE));
            slowdownFactor = constrain(slowdownFactor, SLOWDOWN_MIN_SPEED, 1.0);
            leftSpeed *= slowdownFactor;
            rightSpeed *= slowdownFactor;
        }
        
        if (frontDist < FRONT_STOP_DISTANCE) {
            handleCorner(leftDist, rightDist, frontDist, imu);
            leftSpeed = 0;
            rightSpeed = 0;
            return;
        }
        
        bool leftWallValid = leftDist < WALL_THRESHOLD;
        bool rightWallValid = rightDist < WALL_THRESHOLD;
        
        if (leftWallValid && !rightWallValid) {
            followingLeft = true;
            imu.targetHeading = imu.heading;
        } else if (rightWallValid && !leftWallValid) {
            followingLeft = false;
            imu.targetHeading = imu.heading;
        }
        
        if (followingLeft) {
            followLeftWall(leftSpeed, rightSpeed, leftDist, dt);
        } else {
            followRightWall(leftSpeed, rightSpeed, rightDist, dt);
        }
    }
    
    void handleCorner(float leftDist, float rightDist, float frontDist, IMUHandler &imu) {
        // More conservative corner detection
        bool leftOpen = leftDist > WALL_THRESHOLD * CORNER_OPEN_THRESHOLD;
        bool rightOpen = rightDist > WALL_THRESHOLD * CORNER_OPEN_THRESHOLD;
        bool frontVeryClose = frontDist < FRONT_STOP_DISTANCE * CORNER_CLOSE_FACTOR;
        
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
        turnStartHeading = imu.heading;
        
        // Calculate target heading
        imu.targetHeading = imu.heading + targetTurnAngle;
        
        // Normalize to [-180, 180]
        if (imu.targetHeading > 180.0) imu.targetHeading -= 360.0;
        if (imu.targetHeading < -180.0) imu.targetHeading += 360.0;
        
        integral = 0;
        previousError = 0;
        
        Serial.printf("Start: %.1f° -> Target: %.1f° (Turn: %.1f°)\n", 
                      imu.heading, imu.targetHeading, targetTurnAngle);
    }
    
    bool isTurnComplete(IMUHandler &imu) {
        float headingError = imu.getHeadingError(imu.targetHeading);
        
        // Calculate the actual angle turned
        float turnedAngle = imu.heading - turnStartHeading;
        while (turnedAngle > 180.0) turnedAngle -= 360.0;
        while (turnedAngle < -180.0) turnedAngle += 360.0;
        
        #if DEBUG_TURN_PROGRESS
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > TURN_DEBUG_INTERVAL) {
            Serial.printf("TURN: Cur=%.1f Start=%.1f Targ=%.1f Err=%.1f Turned=%.1f\n", 
                         imu.heading, turnStartHeading, imu.targetHeading, headingError, turnedAngle);
            lastDebug = millis();
        }
        #endif
        
        // Completion conditions
        bool closeToTarget = fabs(headingError) < TURN_ANGLE_TOLERANCE;
        bool turnedEnough = fabs(turnedAngle) >= fabs(targetTurnAngle) * TURN_COMPLETION_FACTOR;
        bool timeout = (millis() - turnStartTime > TURN_TIMEOUT_MS);
        
        return (closeToTarget && turnedEnough) || timeout;
    }
    
    bool isPositioningComplete() {
        return (millis() - positioningStartTime > POSITIONING_TIMEOUT);
    }
    
    void applyHeadingCorrection(int &leftSpeed, int &rightSpeed, IMUHandler &imu) {
        if (currentState != FOLLOWING) return;
        
        float headingError = imu.getHeadingError(imu.targetHeading);
        
        if (abs(headingError) < HEADING_TOLERANCE) return;
        
        float correction = headingError * HEADING_CORRECTION_GAIN;
        correction = constrain(correction, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
        
        leftSpeed += (int)correction;
        rightSpeed -= (int)correction;
    }
    
    void resetPID() {
        integral = 0;
        previousError = 0;
    }
    
    void debugCornerDetection(float leftDist, float rightDist, float frontDist) {
        #if DEBUG_CORNER_DETECTION
        static unsigned long lastDebug = 0;
        if (millis() - lastDebug > CORNER_DEBUG_INTERVAL) {
            bool leftOpen = leftDist > WALL_THRESHOLD;
            bool rightOpen = rightDist > WALL_THRESHOLD;
            bool frontClose = frontDist < FRONT_STOP_DISTANCE;
            
            Serial.printf("CORNER DEBUG: L=%d R=%d F=%d | L_Open=%d R_Open=%d F_Close=%d\n", 
                         (int)leftDist, (int)rightDist, (int)frontDist, 
                         leftOpen, rightOpen, frontClose);
            lastDebug = millis();
        }
        #endif
    }
};

#endif