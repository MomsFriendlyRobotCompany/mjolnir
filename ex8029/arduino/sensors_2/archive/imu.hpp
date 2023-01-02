#pragma once

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD
// #include <Wire.h>
#include <cmath>
#include <squaternion.hpp>
#include <stdint.h>
// #include <cstdint>
#include <yivo.hpp> // MsgIDs
#include "sensors/sensor.hpp" // Sensor, Buffer

/*
F = float
UL = unsigned long (uint32_t)

sensor Size   B   data
----------------------
accel - 3F  (12)  0-2
gyro  - 3F  (12)  3-5
mag   - 3F  (12)  6-8
temp  - 1F  (4)   9
q     - 4F  (16)  10-13
time  - 1UL (4)   14 
-----------------------
total - 15 (60)

timestamp: unsigned long = millis(), overflow in 50 days
*/
class gciLSOXLIS: public Sensor<15> {
  public:

    // gciLSOXLIS()
    //     : found(false), soxFound(false), magFound(false),
    //       bsize(numfloats * sizeof(float)), 
    //       id(IMU_AGMQT),
    //       invg(1.0 / SENSORS_GRAVITY_STANDARD) {}

    gciLSOXLIS(): Sensor<15>(IMU_AGMQT),
        soxFound(false), magFound(false),
        invg(1.0 / SENSORS_GRAVITY_STANDARD) {}

    void init() {
        if (sox.begin_I2C()) {
            soxFound = true;

            // Accelerometer ------------------------------------------
            sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            // sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
            sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

            // Gyros ----------------------------------------------------
            sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            // sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
            sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
        }

        // Magnetometer -----------------------------------------------------
        if (lis3mdl.begin_I2C()) {
            magFound = true;
            lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
            // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
            // already does this
            lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
            // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
            lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        }

        found = magFound && soxFound;
        ts = millis();
    }

    void read() {
        if (soxFound) {
            sox.getEvent(&a, &g, &t);

            data.f[0] = a.acceleration.x * invg;
            data.f[1] = a.acceleration.y * invg;
            data.f[2] = a.acceleration.z * invg;

            data.f[3] = g.gyro.x;
            data.f[4] = g.gyro.y;
            data.f[5] = g.gyro.z;

            data.f[9] = t.temperature;

            uint32_t now = millis();
            dt = (now - ts) * 0.001;
            ts = now;

            Quaternion w(0, data.f[3], data.f[4], data.f[5]);
            q = q + 0.5 * q * w * dt;

            data.f[10] = q.w;
            data.f[11] = q.x;
            data.f[12] = q.y;
            data.f[13] = q.z;

            data.l[14] = now; // time
        }

        if (magFound) {
            lis3mdl.getEvent(&mag);

            data.f[6] = mag.magnetic.x;
            data.f[7] = mag.magnetic.y;
            data.f[8] = mag.magnetic.z;
        }
    }

  protected:
    bool soxFound;
    bool magFound;
    Adafruit_LSM6DSOX sox;    // accel and gyro
    Adafruit_LIS3MDL lis3mdl; // magnetometer
    const float invg;
    sensors_event_t a, g, t;
    sensors_event_t mag;
    Quaternion q;
    float dt;         // time difference between samples
    uint32_t ts; // timestamp (msec)
};
