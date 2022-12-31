// MIT
// Kevin Walchko (c) 2020
// -----------------------------------------------------------
// accel/gyro: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s.html
// mag: https://adafruit.github.io/Adafruit_LIS3MDL/html/class_adafruit___l_i_s3_m_d_l.html
// pressure[lps22]: https://adafruit.github.io/Adafruit_LPS2X/html/_adafruit___l_p_s2_x_8h.html
// pressure[bpm390]: https://github.com/adafruit/Adafruit_BMP3XX/blob/master/bmp3_defs.h
// conversions: https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
// -----------------------------------------------------------

#include <Wire.h>
#include "Arduino.h"

#include <Adafruit_Sensor.h>
#include <GCI_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD
#include <hp_BH1750.h>
//#include <Adafruit_MLX90640.h>

Adafruit_LIS3MDL lis3mdl; // magnetometer
GCI_BMP3XX bmp;           // pressure
Adafruit_LSM6DSOX sox;    // accels/gyros
hp_BH1750 bh1750;         // light
//Adafruit_MLX90640 mlx;    // ir camera
//float frame[32*24];       // buffer for full frame of temperatures
float m[16]; // dummy array for sensor data storage
byte const* p = reinterpret_cast<byte const *>(m);
//byte const* pmlx = reinterpret_cast<byte const *>(frame);
sensors_event_t a;
sensors_event_t g;
sensors_event_t tmp;
sensors_event_t mag;
float ax, ay, az;
float mx, my, mz;
float wx, wy, wz;
float pres, alt;
float temp;
unsigned long start = 0;
unsigned int count = 1;
unsigned int mlxCount = 1;
const float invg = 1.0/SENSORS_GRAVITY_STANDARD;
float lux = 0.0;

bool soxFound = false;
bool lisFound = false;
bool bmpFound = false;
bool bh1750Found = false;
bool mlxFound = false;

uint8_t messageLength = 2; // count [EE,EE] at end

void setup(void) {
    Serial.begin(1000000); // 1Mbps
//    Serial.begin(115200);
    Serial.setTimeout(1);
    while (!Serial)
        delay(1000); // will pause Zero, Leonardo, etc until serial console opens

    Wire.setClock(400000); // 400 kHz

    if (sox.begin_I2C()) {
        soxFound = true;
        messageLength += 24; // 6f * 4B = 24B
        messageLength += 1; // header
        
        // Accelerometer ------------------------------------------
        sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
        sox.setAccelDataRate(LSM6DS_RATE_208_HZ);

        // Gyros ----------------------------------------------------
        sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
        sox.setGyroDataRate(LSM6DS_RATE_208_HZ);
    }

    // Magnetometer -----------------------------------------------------
      if (lis3mdl.begin_I2C()) {
          lisFound = true;
          messageLength += 12; // 3f * 4B = 12 B
//          messageLength += 1; // header - this is captured with the accel/gyro
          // lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already does this
          // lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300 already does this
          lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
          lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
          lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
      }

    // Pressure/Temperature -----------------------------------------------
    if (bmp.begin_I2C()) {
        bmpFound = true;
        messageLength += 8; // 2f * 4B = 8B
        messageLength += 1; // header

        // Set up oversampling and filter initialization
        // Datasheet, table 9, pg 17
        if (1) {
            // Drone
            // Mode: Normal
            // OS Pres: x8
            // OS Temp: x1
            // IIR: 2 -> 3
            // Sampling: 50Hz
            // RMS Noise [cm]: 11
            bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
            bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
            bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
            bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        }
        else {
            // Indoor nav
            // Mode: Normal
            // OS Pres: x16
            // OS Temp: x2
            // IIR: 4 -> 15
            // Sampling: 25Hz
            // RMS Noise [cm]: 5
            bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
            bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
            bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
            bmp.setOutputDataRate(BMP3_ODR_25_HZ);
        }
    }

    // Light ----------------------------------------------------------------
    if (bh1750.begin(BH1750_TO_GROUND)) {
        bh1750Found = true;
        messageLength += 4; // 1f * 4B = 4B
        messageLength += 1; // header
        bh1750.calibrateTiming();  // calibrate the timings, about 855ms with a bad chip
        bh1750.start();            // start the first measurement
    }

    // IR -------------------------------------------------------------------
    // WARNING: this driver is buggy and has a beat that slows everything
    //          down for some reason.
//    if (mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
//        mlxFound = true;
//        //mlx.setMode(MLX90640_INTERLEAVED);
//        mlx.setMode(MLX90640_CHESS);
//        mlx.setResolution(MLX90640_ADC_18BIT);
//        mlx.setRefreshRate(MLX90640_64_HZ);
//    }
    
}

/*
[0xFF,0xFF]: start
0xFE: accel, gyro
0xFD: accel, gyro, mag
0xFC: mag
0xFB: temperature, pressure
0xFA: 
0xF9: light
0xF8: MLX90640 IR camera
0xF7-0xF1: unused
0xF0: position, velocity, quaternion
[0xEE,0xEE]: end
*/

uint8_t header[2] = {0xFF,0xFF};
uint8_t ender[2] = {0xEE,0xEE};
float hz = 0.0;
bool hzToggle = false;

void loop() {
    int numChar = Serial.available();
    int b = 0;
    if (numChar > 0) b = Serial.read();

    // Enable displaying loop rate
    if (b == 'h'){
        hzToggle = !hzToggle;
        start = micros();
        count = 1;
    }
    // update or print debugging loop rate info
    if (hzToggle){
        if (count % 10 == 0){
            unsigned long now = micros();
            hz = 1000000.0*float(count+1)/float(now - start);
            Serial.println(hz,2);
            count = 1;
            start = now;
        }
        else {
            count += 1;
        }
    }

//    if (mlxFound && 0){
//        if (mlxCount >= 100){
//            mlx.getFrame(frame);
//            mlxCount = 0;
//        }
//        else {
//            mlxCount +=1;
//        }
//        if (b == 'm') {
//            Serial.write(header, 2);
//            Serial.write(0xf8);
//            Serial.write(pmlx, sizeof(frame));
//            Serial.write(ender, 2);
//            return;
//        }
//    }

    if (soxFound) {
        sox.getEvent(&a,&g,&tmp);

        ax = a.acceleration.x * invg;
        ay = a.acceleration.y * invg;
        az = a.acceleration.z * invg;

        wx = g.gyro.x;
        wy = g.gyro.y;
        wz = g.gyro.z;
    }

    if (lisFound){
        lis3mdl.getEvent(&mag);

        mx = mag.magnetic.x;
        my = mag.magnetic.y;
        mz = mag.magnetic.z;
    }

    if (bmpFound) {
        bmp.performReading();
        pres = bmp.pressure;
        temp = bmp.temperature; // C
    }

    if (bh1750Found){
       if (bh1750.hasValue() == true) {    // non blocking reading
           lux = bh1750.getLux();
           bh1750.start();
       }
   }

    // Send data
    // [FF,FF,msgLen, ...., EE,EE]
    if (b == 'g') {
        Serial.write(header, 2);
        Serial.write(messageLength);

        if (soxFound && lisFound){
            Serial.write(0xFD);
            m[0] = ax; // m/sec^2
            m[1] = ay;
            m[2] = az;

            m[3] = wx; // rads/sec
            m[4] = wy;
            m[5] = wz;

            m[6] = mx;  // uT
            m[7] = my;
            m[8] = mz;
            Serial.write(p, 36);
        }
        else {
            if (soxFound){
                Serial.write(0xFE);
                m[0] = ax; // m/sec^2
                m[1] = ay;
                m[2] = az;
    
                m[3] = wx; // rads/sec
                m[4] = wy;
                m[5] = wz;
                Serial.write(p, 24);
            }
            else if (lisFound){
                Serial.write(0xFC);
                m[0] = mx; // uT
                m[1] = my;
                m[2] = mz;
                Serial.write(p, 12);
            }
        }
        
        if (bmpFound){
            Serial.write(0xFB);
            m[0] = temp; // C
            m[1] = pres; // Pa
            Serial.write(p, 8);
        }

        if (bh1750Found){
            Serial.write(0xF9);
            m[0] = lux; // lux
            Serial.write(p, 4);
        }

        Serial.write(ender, 2);
    }
}
