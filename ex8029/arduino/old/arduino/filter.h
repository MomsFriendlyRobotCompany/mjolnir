#pragma once

#include "Arduino.h"

class Madgwick {
public:
    Madgwick(const float beta);

    void getEuler(float &roll, float &pitch, float &yaw);
    void update(float gx, float gy, float gz);
    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

    float q[4];
    
 protected:
    float beta;
    float Now, lastUpdate, deltat;
};

class eCompass {
public:
    void getEuler(float &roll, float &pitch, float &yaw);
    void update(float ax, float ay, float az, float mx, float my, float mz);

    float roll, pitch, yaw;
};
