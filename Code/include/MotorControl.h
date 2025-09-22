#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "config.h"

class MotorControl {
private:
    int currentLeftSpeed = 0;
    int currentRightSpeed = 0;
    int turnCurrentLeft = 0;  // Separate tracking for turns
    int turnCurrentRight = 0;

public:
    void setup() {
        pinMode(AIN1, OUTPUT);
        pinMode(AIN2, OUTPUT);
        pinMode(BIN1, OUTPUT);
        pinMode(BIN2, OUTPUT);
        ledcSetup(0, 3000, 8);
        ledcSetup(1, 3000, 8);
        ledcAttachPin(PWMA, 0);
        ledcAttachPin(PWMB, 1);
        
        // Initialize turn velocities
        turnCurrentLeft = 0;
        turnCurrentRight = 0;
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
    
    void stopMotors() {
        currentLeftSpeed = 0;
        currentRightSpeed = 0;
        turnCurrentLeft = 0;
        turnCurrentRight = 0;
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
        ledcWrite(0, 0);
        ledcWrite(1, 0);
    }
    
    void applyTurnVelocityProfile(int &leftSpeed, int &rightSpeed, int targetLeft, int targetRight) {
        // Apply acceleration/deceleration to turn speeds
        if (turnCurrentLeft < targetLeft) {
            turnCurrentLeft = min(turnCurrentLeft + TURN_ACCELERATION, targetLeft);
        } else if (turnCurrentLeft > targetLeft) {
            turnCurrentLeft = max(turnCurrentLeft - TURN_ACCELERATION, targetLeft);
        }
        
        if (turnCurrentRight < targetRight) {
            turnCurrentRight = min(turnCurrentRight + TURN_ACCELERATION, targetRight);
        } else if (turnCurrentRight > targetRight) {
            turnCurrentRight = max(turnCurrentRight - TURN_ACCELERATION, targetRight);
        }
        
        leftSpeed = turnCurrentLeft;
        rightSpeed = turnCurrentRight;
    }
    
    void setTurnMotors(int leftSpeed, int rightSpeed) {
        digitalWrite(AIN1, leftSpeed > 0 ? HIGH : LOW);
        digitalWrite(AIN2, leftSpeed > 0 ? LOW : HIGH);
        ledcWrite(0, abs(leftSpeed));

        digitalWrite(BIN1, rightSpeed > 0 ? LOW : HIGH);
        digitalWrite(BIN2, rightSpeed > 0 ? HIGH : LOW);
        ledcWrite(1, abs(rightSpeed));
        
        currentLeftSpeed = leftSpeed;
        currentRightSpeed = rightSpeed;
    }
    
    void resetTurnProfile() {
        turnCurrentLeft = 0;
        turnCurrentRight = 0;
    }
    
    int getCurrentLeftSpeed() { return currentLeftSpeed; }
    int getCurrentRightSpeed() { return currentRightSpeed; }
};

#endif
