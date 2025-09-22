#ifndef IMUHANDLER_H
#define IMUHANDLER_H

#include "config.h"

class IMUHandler {
private:
    float gyroZeroOffset = 0.0;
    unsigned long lastStillTime = 0;
    unsigned long lastUpdate = 0;

public:
    float heading = 0.0;
    float targetHeading = 0.0;
    bool headingInitialized = false;
    int16_t ax, ay, az, gx, gy, gz;
    
    void updateHeading(int16_t gz_val, unsigned long currentTime, RobotState currentState) {
        static float gyroBias = 0.0;
        float dt = (currentTime - lastUpdate) / 1000.0;
        
        if (dt > 0 && dt < MAX_DT) {
            float gyroZ = (gz_val / GYRO_SENSITIVITY) - gyroBias;
            heading += gyroZ * dt;
            
            // Auto-calibrate gyro bias when robot is still
            if (currentState == FOLLOWING && abs(gyroZ) < 1.0) {
                if (currentTime - lastStillTime > STILL_TIME_THRESHOLD) {
                    gyroBias = gyroBias * 0.99 + (gz_val / GYRO_SENSITIVITY) * 0.01;
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
    
    void initializeHeading() {
        headingInitialized = false;
    }
    
    float getHeadingError(float target) {
        float error = target - heading;
        while (error > 180.0) error -= 360.0;
        while (error < -180.0) error += 360.0;
        return error;
    }
};

#endif