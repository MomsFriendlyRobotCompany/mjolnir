#pragma once
#include <Wire.h>
#include <stdint.h>

#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

enum class Sensors {
    SOXLIS = 0xF1,
    S33LIS = 0xF0,
    DPS310 = 0xB1
};

class gciLSOXLIS {
public:

    static const uint8_t id = uint8_t(Sensors::SOXLIS);

    gciLSOXLIS(): found(false), soxFound(false), magFound(false),
//        pbuffer(reinterpret_cast<byte const *>(buffer)),
        bsize(10*sizeof(float)),
        invg(1.0/SENSORS_GRAVITY_STANDARD) {;}

    void init(){
        if (sox.begin_I2C()) {
            soxFound = true;

            // Accelerometer ------------------------------------------
            sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
            sox.setAccelDataRate(LSM6DS_RATE_208_HZ);

            // Gyros ----------------------------------------------------
            sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
            sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
        }

        // Magnetometer -----------------------------------------------------
        if (lis3mdl.begin_I2C()) {
          magFound = true;
          // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already does this
          // lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300 already does this
          lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
          lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
          lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
        }

        found = magFound && soxFound;
    }

    void read(){
        if (soxFound) {
            sox.getEvent(&a,&g,&t);

            data.f[0] = a.acceleration.x * invg;
            data.f[1] = a.acceleration.y * invg;
            data.f[2] = a.acceleration.z * invg;

            data.f[3] = g.gyro.x;
            data.f[4] = g.gyro.y;
            data.f[5] = g.gyro.z;

            data.f[9] = t.temperature;
        }

        if (magFound){
            lis3mdl.getEvent(&mag);

            data.f[6] = mag.magnetic.x;
            data.f[7] = mag.magnetic.y;
            data.f[8] = mag.magnetic.z;
        }
    }

    bool found;

    union { byte b[10*sizeof(float)]; float f[10]; } data;
    const uint8_t bsize; // length of array

protected:
    bool soxFound;
    bool magFound;
    Adafruit_LSM6DSOX sox; // accel and gyro
    Adafruit_LIS3MDL lis3mdl; // magnetometer
    const float invg;
    sensors_event_t a,g,t;
    sensors_event_t mag;
};

/////////////////////////////////////////////////////////////////////////////////////

#include <Adafruit_DPS310.h>

class gciDPS310 {
public:

    static const uint8_t id = uint8_t(Sensors::DPS310);

    gciDPS310(): bsize(2*sizeof(float)), found(false) {;}

    // Sets up the sensor
    void init(){
        if (dps.begin_I2C()) {
            dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
            dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
            found = true;
        }
    }

    void read(){
        if (found){
            if (dps.temperatureAvailable() || dps.pressureAvailable()) {
                dps.getEvents(&temp_event, &pressure_event);
                data.f[0] = temp_event.temperature;
                data.f[1] = pressure_event.pressure;
            }
        }
    }

    bool found;

    union { byte b[2*sizeof(float)]; float f[2]; } data;
    const uint8_t bsize; // length of array

protected:
    Adafruit_DPS310 dps; // pressure / temperature
    sensors_event_t temp_event, pressure_event;
};


/////////////////////////////////////////////////////////////////////////////////////
// not sure about this!!!!!
// bn085 uses a callback method to get data from the sensor ... wtf?
// I can't guarentee reading data at any time and it ONLY returns the
// reading for 1 data point (i.e., accell, gyro, mag, etc) for every
// reading. Has to be a better way to force a common sampling of the
// sensors each time ... I don't even get to choose: read gyro! It
// determines what data to send me ... total crap!


//#include <Adafruit_BNO08x.h>
//
//class gciB085 {
//public:
//    static const uint8_t id = uint8_t(Sensors::B085);
//
//    gciB085(): bsize(x*sizeof(float), found(false) {;}
//
//    void init(){
//        bno08x.begin_I2C();
//    }
//
//    void read(){
//        if (bno08x.wasReset()) {
//            setReports();
//        }
//    }
//
//    bool found;
//
//protected:
//
//    void setReports(){
//
//    }
//    Adafruit_BNO08x bno08x(-1);
//
//}
