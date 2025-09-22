#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Wire.h>
#include <VL53L0X.h>
#include <MPU6050.h>
#include "config.h"

class SensorManager {
private:
    // TOF Sensors
    VL53L0X sensorLeft;
    VL53L0X sensorRight;
    VL53L0X sensorFront;
    
    // MPU6050
    MPU6050 mpu;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    // Sensor smoothing
    int leftDistHistory[SENSOR_FILTER_SIZE];
    int rightDistHistory[SENSOR_FILTER_SIZE];
    int frontDistHistory[SENSOR_FILTER_SIZE];
    
    // Low-pass filter
    float filteredLeftDist = 0;
    float filteredRightDist = 0;
    float filteredFrontDist = 0;

public:
    bool setup() {
        // Initialize TOF Sensors
        pinMode(XSHUT_LEFT, OUTPUT);
        pinMode(XSHUT_RIGHT, OUTPUT);
        pinMode(XSHUT_FRONT, OUTPUT);
        
        digitalWrite(XSHUT_LEFT, LOW);
        digitalWrite(XSHUT_RIGHT, LOW);
        digitalWrite(XSHUT_FRONT, LOW);
        delay(50);

        digitalWrite(XSHUT_LEFT, HIGH);
        delay(15);
        if(!sensorLeft.init()) { 
            Serial.println("Left sensor failed!"); 
            return false;
        }
        sensorLeft.setAddress(0x30);
        sensorLeft.setTimeout(500);
        sensorLeft.startContinuous();
        sensorLeft.setMeasurementTimingBudget(20000);
        
        digitalWrite(XSHUT_RIGHT, HIGH);
        delay(15);
        if(!sensorRight.init()) { 
            Serial.println("Right sensor failed!"); 
            return false;
        }
        sensorRight.setAddress(0x32);
        sensorRight.setTimeout(500);
        sensorRight.startContinuous();
        sensorRight.setMeasurementTimingBudget(20000);
        
        digitalWrite(XSHUT_FRONT, HIGH);
        delay(15);
        if(!sensorFront.init()) { 
            Serial.println("Front sensor failed!"); 
            return false;
        }
        sensorFront.setAddress(0x34);
        sensorFront.setTimeout(500);
        sensorFront.startContinuous();
        sensorFront.setMeasurementTimingBudget(20000);

        // Initialize sensor history
        for (int i = 0; i < SENSOR_FILTER_SIZE; i++) {
            leftDistHistory[i] = DESIRED_DISTANCE;
            rightDistHistory[i] = DESIRED_DISTANCE;
            frontDistHistory[i] = MAX_SENSOR_RANGE;
        }

        // Initialize MPU6050
        mpu.initialize();
        delay(100);
        if (mpu.testConnection()) {
            Serial.println("MPU6050 connection successful!");
            // Calibration
            Serial.println("Calibrating MPU6050... keep robot still!");
            mpu.CalibrateAccel(CALIBRATION_SAMPLES);
            mpu.CalibrateGyro(CALIBRATION_SAMPLES);
            Serial.println("Calibration complete!");
            return true;
        } else {
            Serial.println("MPU6050 connection failed!");
            return false;
        }
    }
    
    void readSensors(float &leftDist, float &rightDist, float &frontDist) {
        leftDist = getFilteredDistance(sensorLeft, filteredLeftDist);
        rightDist = getFilteredDistance(sensorRight, filteredRightDist);
        frontDist = getFilteredDistance(sensorFront, filteredFrontDist);
    }
    
    void readMPU6050(int16_t &ax_out, int16_t &ay_out, int16_t &az_out, 
                     int16_t &gx_out, int16_t &gy_out, int16_t &gz_out) {
        if (mpu.testConnection()) {
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            ax_out = ax;
            ay_out = ay;
            az_out = az;
            gx_out = gx;
            gy_out = gy;
            gz_out = gz;
        }
    }
    
    float getFilteredDistance(VL53L0X &sensor, float &filteredValue) {
        int raw = sensor.readRangeContinuousMillimeters();
        if (sensor.timeoutOccurred() || raw > MAX_SENSOR_RANGE) raw = MAX_SENSOR_RANGE;
        
        filteredValue = SENSOR_FILTER_ALPHA * raw + (1 - SENSOR_FILTER_ALPHA) * filteredValue;
        return filteredValue;
    }
};

#endif