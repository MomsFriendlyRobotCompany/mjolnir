#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <squaternion.hpp>
#include <gciSensors.hpp>
#include <cstdint>
#include <msgids.hpp> // MsgIDs
#include "sensor.hpp"


/*
This is a wrapper around a group of sensors running approx
at the same update rate. All data is stored in a buffer
and sent to a computer using Serial.write(buff, leng)
command.

F = float
UL = unsigned long (uint32_t)

sensor     Size   B   data    Units
-------------------------------------
accel     - 3F  (12)  0-2    g
gyro      - 3F  (12)  3-5    rad/s
imu temp  - 1F  (4)   6      C
mag       - 3F  (12)  7-9    uT
q         - 4F  (16)  10-13  
press     - 1F  (4)   14     Pa
air temp  - 1F  (4)   15     C
time      - 1UL (4)   16     sec
-------------------------------------
total     - 17 (xx)  x 100 Hz x 8b = xx00 bps = xx kbps

Overhead: 6B x 8b x 100 Hz = 4,800 bps = 4.8 kbps

Motors: 4F x 4B x 8b x 100 Hz = 12,800 bps = 12.8 kbps

timestamp: unsigned long = millis(), overflow in 50 days
*/

#if 1

// ax,ay,az    g
// wx,wy,wz    rad/sec
// temperature C
// time stamp  millisecond
constexpr uint8_t NUMM_FLOATS = 8;
#define NO_CORRECTIONS true

class gciLSOX: public mSensor<NUMM_FLOATS> {
  public:
    bool soxFound;
    bool lisFound;
    bool bmpFound;

    gciLSOX(): mSensor<NUMM_FLOATS>(IMU_AGT),
        soxFound(false),
        // sm{{ 1.00268927, -0.00056029, -0.00190925, -0.00492348},
        //     {-0.00138898,  0.99580818, -0.00227335,  0.00503835},
        //     {-0.01438271,  0.00673172,  0.9998954 , -0.01364759}},
        // gbias{-0.00889949 -0.00235061 -0.00475294},
        lisFound(false),
        // mbias{-13.15340002, 29.7714855, 0.0645215},
        // mm{0.96545537,0.94936676,0.967698},
        bmpFound(false),
        sox(&Wire) {}

    void init() {
        if (sox.init()) {
            soxFound = true;

            // Accelerometer ------------------------------------------
            // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            // sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

            // Gyros ----------------------------------------------------
            // sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            // sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
        }

        // Magnetometer -----------------------------------------------------
        // if (lis3mdl.init()) {
        //     magFound = true;
        //     // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
        //     // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
        //     // already does this
        //     // lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
        //     // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
        //     // lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        // }

        // if (bmp.init()) {
        //   pressFound = true;
        // }

        found = soxFound;
        // ts = millis();
    }

    void read() {
        if (soxFound) {
            LSM6DSOX::sox_t s = sox.read();

            #if NO_CORRECTIONS
            data.f[0] = s.ax; // g
            data.f[1] = s.ay; 
            data.f[2] = s.az;
            #else
            data.f[0] = sm[0][0] * s.ax + sm[0][1] * s.ay + sm[0][2] * s.az + sm[0][3];
            data.f[1] = sm[1][0] * s.ax + sm[1][1] * s.ay + sm[1][2] * s.az + sm[1][3];
            data.f[2] = sm[2][0] * s.ax + sm[2][1] * s.ay + sm[2][2] * s.az + sm[2][3];
            #endif

            #if NO_CORRECTIONS
            data.f[3] = s.gx; // rad/s
            data.f[4] = s.gy;
            data.f[5] = s.gz;
            #else
            data.f[3] = (s.gx - gbias[0]);
            data.f[4] = (s.gy - gbias[1]);
            data.f[5] = (s.gz - gbias[2]);
            #endif

            data.f[6] = s.temp; // C

            uint32_t now = millis();
            // dt = (now - ts) * 0.001;
            // ts = now;

            // q = qcf.update(data.f[0], data.f[1], data.f[2],data.f[3], data.f[4], data.f[5], dt);

            // data.f[10] = q.w;
            // data.f[11] = q.x;
            // data.f[12] = q.y;
            // data.f[13] = q.z;

            data.l[7] = now; // time msec
        }

        // if (magFound) {
        //     LIS3MDL::mag_t s = lis3mdl.read(); 

        //     #if NO_CORRECTIONS
        //     data.f[7] = s.x; // uT
        //     data.f[8] = s.y;
        //     data.f[9] = s.z;
        //     #else
        //     data.f[7] = mm[0] * s.x - mbias[0]; // uT
        //     data.f[8] = mm[1] * s.y - mbias[1];
        //     data.f[9] = mm[2] * s.z - mbias[2];
        //     #endif            
        // }

        // if (pressFound) {
        //   BMP390::pt_t s = bmp.read();
        //   if (s.ok) {
        //     data.f[14] = s.press;
        //     data.f[15] = s.temp;
        //   }
        //   else {
        //     data.f[14] = 0.0f;
        //     data.f[15] = 0.0f;
        //   }
        // }
    }

  protected:
    // bool soxFound;
    // bool lisFound;
    // bool bmpFound;

    // QCF qcf;

    LSM6DSOX::gciLSM6DSOX sox;   // accel and gyro
    // LIS3MDL::gciLIS3MDL lis3mdl; // magnetometer
    // BMP390::gciBMP390 bmp;       // pressure
    // Quaternion q;
    // float dt;       // time difference between samples
    // uint32_t ts;    // timestamp (msec)
    // float sm[3][4]; // accel scale/bias
    // float gbias[3]; // gyro bias
    // float mbias[3]; // mag bias
    // float mm[3];    // mag scale
};
#endif


// #if 0
// constexpr uint8_t imuNumFloats = 17;

// class gciLSOXLISBMP: public mSensor<imuNumFloats> {
//   public:

//     gciLSOXLISBMP(): mSensor<imuNumFloats>(IMU_AGMQPT),
//         soxFound(false), magFound(false), pressFound(false),
//         qcf(0.02),
//         sm{{ 1.00268927, -0.00056029, -0.00190925, -0.00492348},
//             {-0.00138898,  0.99580818, -0.00227335,  0.00503835},
//             {-0.01438271,  0.00673172,  0.9998954 , -0.01364759}},
//         gbias{-0.00889949 -0.00235061 -0.00475294},
//         mbias{-13.15340002, 29.7714855, 0.0645215},
//         mm{0.96545537,0.94936676,0.967698},
//         sox(&Wire), lis3mdl(&Wire), bmp(&Wire) {}

//     void init() {
//         if (sox.init()) {
//             soxFound = true;

//             // Accelerometer ------------------------------------------
//             // sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
//             // sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

//             // Gyros ----------------------------------------------------
//             // sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
//             // sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
//         }

//         // Magnetometer -----------------------------------------------------
//         if (lis3mdl.init()) {
//             magFound = true;
//             // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
//             // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
//             // already does this
//             // lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
//             // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
//             // lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
//         }

//         if (bmp.init()) {
//           pressFound = true;
//         }

//         found = magFound && soxFound && pressFound;
//         ts = millis();
//     }

//     void read() {
//         if (soxFound) {
//             LSM6DSOX::sox_t s = sox.read();

//             #if 0
//             data.f[0] = s.ax; // g
//             data.f[1] = s.ay; 
//             data.f[2] = s.az;
//             #else
//             data.f[0] = sm[0][0] * s.ax + sm[0][1] * s.ay + sm[0][2] * s.az + sm[0][3];
//             data.f[1] = sm[1][0] * s.ax + sm[1][1] * s.ay + sm[1][2] * s.az + sm[1][3];
//             data.f[2] = sm[2][0] * s.ax + sm[2][1] * s.ay + sm[2][2] * s.az + sm[2][3];
//             #endif

//             #if 0
//             data.f[3] = s.gx; // rad/s
//             data.f[4] = s.gy;
//             data.f[5] = s.gz;
//             #else
//             data.f[3] = (s.gx - gbias[0]);
//             data.f[4] = (s.gy - gbias[1]);
//             data.f[5] = (s.gz - gbias[2]);
//             #endif

//             data.f[6] = s.temp; // C

//             uint32_t now = millis();
//             dt = (now - ts) * 0.001;
//             ts = now;

//             q = qcf.update(data.f[0], data.f[1], data.f[2],data.f[3], data.f[4], data.f[5], dt);

//             data.f[10] = q.w;
//             data.f[11] = q.x;
//             data.f[12] = q.y;
//             data.f[13] = q.z;

//             data.l[16] = now; // time msec
//         }

//         if (magFound) {
//             LIS3MDL::mag_t s = lis3mdl.read(); 

//             #if 0
//             data.f[7] = s.x; // uT
//             data.f[8] = s.y;
//             data.f[9] = s.z;
//             #else
//             data.f[7] = mm[0] * s.x - mbias[0]; // uT
//             data.f[8] = mm[1] * s.y - mbias[1];
//             data.f[9] = mm[2] * s.z - mbias[2];
//             #endif            
//         }

//         if (pressFound) {
//           BMP390::pt_t s = bmp.read();
//           if (s.ok) {
//             data.f[14] = s.press;
//             data.f[15] = s.temp;
//           }
//           else {
//             data.f[14] = 0.0f;
//             data.f[15] = 0.0f;
//           }
//         }
//     }

//   protected:
//     bool soxFound;
//     bool magFound;
//     bool pressFound;

//     QCF qcf;

//     LSM6DSOX::gciLSM6DSOX sox;   // accel and gyro
//     LIS3MDL::gciLIS3MDL lis3mdl; // magnetometer
//     BMP390::gciBMP390 bmp;       // pressure
//     Quaternion q;
//     float dt;       // time difference between samples
//     uint32_t ts;    // timestamp (msec)
//     float sm[3][4]; // accel scale/bias
//     float gbias[3]; // gyro bias
//     float mbias[3]; // mag bias
//     float mm[3];    // mag scale
// };
// #endif











// #if 0 /////////////////////////////////////////////////////////////////////////////////

// #include <Adafruit_LIS3MDL.h>
// #include <Adafruit_LSM6DSOX.h>
// #include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD
// // #include <Wire.h>
// // #include <cmath>
// #include <squaternion.hpp>
// // #include <stdint.h>
// // #include <cstdint>
// // #include <yivo.hpp> // MsgIDs
// #include "sensor.hpp" // Sensor, Buffer




// /*
// F = float
// UL = unsigned long (uint32_t)

// sensor Size   B   data
// ----------------------
// accel - 3F  (12)  0-2
// gyro  - 3F  (12)  3-5
// mag   - 3F  (12)  6-8
// q     - 4F  (16)  9-12
// temp  - 1F  (4)   13
// time  - 1UL (4)   14 
// -----------------------
// total - 15 (60)

// timestamp: unsigned long = millis(), overflow in 50 days
// */
// class gciLSOXLIS: public mSensor<15> {
//   public:

//     // gciLSOXLIS()
//     //     : found(false), soxFound(false), magFound(false),
//     //       bsize(numfloats * sizeof(float)), 
//     //       id(IMU_AGMQT),
//     //       invg(1.0 / SENSORS_GRAVITY_STANDARD) {}

//     gciLSOXLIS(): mSensor<15>(IMU_AGMQT),
//         soxFound(false), magFound(false),
//         invg(1.0 / SENSORS_GRAVITY_STANDARD) {}

//     void init() {
//         if (sox.begin_I2C()) {
//             soxFound = true;

//             // Accelerometer ------------------------------------------
//             sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
//             // sox.setAccelDataRate(LSM6DS_RATE_208_HZ);
//             sox.setAccelDataRate(LSM6DS_RATE_104_HZ);

//             // Gyros ----------------------------------------------------
//             sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
//             // sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
//             sox.setGyroDataRate(LSM6DS_RATE_104_HZ);
//         }

//         // Magnetometer -----------------------------------------------------
//         if (lis3mdl.begin_I2C()) {
//             magFound = true;
//             lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already
//             // does this lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300
//             // already does this
//             lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
//             // lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
//             lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
//         }

//         found = magFound && soxFound;
//         ts = millis();
//     }

//     void read() {
//         if (soxFound) {
//             sox.getEvent(&a, &g, &t);

//             data.f[0] = a.acceleration.x * invg;
//             data.f[1] = a.acceleration.y * invg;
//             data.f[2] = a.acceleration.z * invg;

//             data.f[3] = g.gyro.x;
//             data.f[4] = g.gyro.y;
//             data.f[5] = g.gyro.z;

//             data.f[13] = t.temperature;

//             uint32_t now = millis();
//             dt = (now - ts) * 0.001;
//             ts = now;

//             Quaternion w(0, data.f[3], data.f[4], data.f[5]);
//             q = q + 0.5 * q * w * dt;

//             data.f[9] = q.w;
//             data.f[10] = q.x;
//             data.f[11] = q.y;
//             data.f[12] = q.z;

//             data.l[14] = now; // time
//         }

//         if (magFound) {
//             lis3mdl.getEvent(&mag);

//             data.f[6] = mag.magnetic.x;
//             data.f[7] = mag.magnetic.y;
//             data.f[8] = mag.magnetic.z;
//         }
//     }

//   protected:
//     bool soxFound;
//     bool magFound;
//     Adafruit_LSM6DSOX sox;    // accel and gyro
//     Adafruit_LIS3MDL lis3mdl; // magnetometer
//     const float invg;
//     sensors_event_t a, g, t;
//     sensors_event_t mag;
//     Quaternion q;
//     float dt;         // time difference between samples
//     uint32_t ts; // timestamp (msec)
// };

// #endif
