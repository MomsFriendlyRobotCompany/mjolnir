// MIT
// Kevin Walchko (c) 2020
// -----------------------------------------------------------
// accel/gyro: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s.html
// mag: https://adafruit.github.io/Adafruit_LIS3MDL/html/class_adafruit___l_i_s3_m_d_l.html
// pressure[lps22]: https://adafruit.github.io/Adafruit_LPS2X/html/_adafruit___l_p_s2_x_8h.html
// pressure[bpm390]: https://github.com/adafruit/Adafruit_BMP3XX/blob/master/bmp3_defs.h
// conversions: https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
// -----------------------------------------------------------
/*
IMU: 12*8 = 96
IR: 24*32*8 = 6144

*/
#include <Wire.h>
#include "Arduino.h"

#include <Adafruit_Sensor.h>
#include <GCI_BMP3XX.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD
#include <hp_BH1750.h>

Adafruit_LIS3MDL lis3mdl; // magnetometer
GCI_BMP3XX bmp;           // pressure
Adafruit_LSM6DSOX sox;    // accels/gyros
hp_BH1750 bh1750;         // light

// Header (0xFF) adds 1 to m array
// float m[18]; // // accel(3), gyro(3), mag(3), pressure, temperature,bno.q(4),bno.euler(3)
 float m[12]; // // accel(3), gyro(3), mag(3), pressure, temperature, light
// float m[11]; // // accel(3), gyro(3), mag(3), pressure, temperature
//float m[8]; // accel(3), gyro(3), pressure, temperature
byte const* p = reinterpret_cast<byte const *>(m);
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
const float invg = 1.0/SENSORS_GRAVITY_STANDARD;
float lux = 0.0;


void setup(void) {
    Serial.begin(1000000); // 1Mbps
//    Serial.begin(115200);
    Serial.setTimeout(1);
    while (!Serial)
        delay(1000); // will pause Zero, Leonardo, etc until serial console opens

    Wire.setClock(400000); // 400 kHz

    if (!sox.begin_I2C()) {
        while (1) {
          delay(1000);
          Serial.println("LSM6DSOX NOT Found!");
        }
    }
    // Accelerometer ------------------------------------------
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    sox.setAccelDataRate(LSM6DS_RATE_208_HZ);

    // Gyros ----------------------------------------------------
    sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    sox.setGyroDataRate(LSM6DS_RATE_208_HZ);

    // Magnetometer -----------------------------------------------------
      if (!lis3mdl.begin_I2C()) {
        while (1) {
            delay(1000);
            Serial.println("LIS3MDL NOT Found!");
        }
      }
//      lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already does this
//      lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300 already does this
      lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
      lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ); // sets LIS3MDL_HIGHMODE
      lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    // Pressure/Temperature -----------------------------------------------
    if (!bmp.begin_I2C()) {
        while (1){
            Serial.println("BMP3 NOT Found!");
            delay(1000);
        }
    }

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

    // Light ----------------------------------------------------------------
//    bh1750.begin(BH1750_TO_GROUND);
//    if (!bh1750.begin(BH1750_TO_GROUND)) { // set address and init sensor
//        while (1){
//            Serial.println("BH1750 NOT Found!");
//            delay(1000);
//        }
//    }
//    bh1750.calibrateTiming();       // calibrate the timings, about 855ms with a bad chip
//    bh1750.start();                 // start the first measurement

    start = micros();
}


void loop() {
    unsigned long loop_start = micros();
//    bool freeRun = false;
    int b = 0;

    sox.getEvent(&a,&g,&tmp);
    lis3mdl.getEvent(&mag);
    bmp.performReading();

    ax = a.acceleration.x * invg;
    ay = a.acceleration.y * invg;
    az = a.acceleration.z * invg;

    wx = g.gyro.x;
    wy = g.gyro.y;
    wz = g.gyro.z;

    mx = mag.magnetic.x;
    my = mag.magnetic.y;
    mz = mag.magnetic.z;

//    if (bh1750.hasValue() == true) {    // non blocking reading
//        lux = bh1750.getLux();
//        bh1750.start();
//    }

    pres = bmp.pressure;
    // temp = tmp.temperature; // C - faster?
    temp = bmp.temperature; // C
    // temp = bmp.temperature*9.0/5.0+32.0; // F
    // alt = bmp.readAltitude(SENSORS_PRESSURE_SEALEVELHPA); // m

    if (0 && (count % 10 == 0)){
        unsigned long now = micros();
        float hz = 1000000.0*float(count+1)/float(now - start);
        Serial.println(hz,2);
//        Serial.println(pres,2);
//        Serial.print(wy,2);
//        Serial.print(",");
//        Serial.println(wx,2);
//        Serial.println(lux,4);
        count = 1;
        start = now;
    }
    else {
        count += 1;
    }

    // chew through input buffer looking for 'g'
    int numChar = Serial.available();
    b = 0;
    if (numChar > 0) {
        for (int i=0; i<numChar; i++)
            if (Serial.read() == 'g') b = 'g';
    }
    if (b == 'g') {
        m[0] = ax; // m/sec^2
        m[1] = ay;
        m[2] = az;

        m[3] = wx; // rads/sec
        m[4] = wy;
        m[5] = wz;

        m[6] = mx;  // uT
        m[7] = my;
        m[8] = mz;

        m[9] = pres; // Pa
        m[10] = temp; // C

//        m[11] = lux; // lux

        Serial.write(0xff);
        Serial.write(p, sizeof(m));
//        Serial.write('\n');
    }

    // 5000  -> 200Hz
    // 20000 -> 50Hz
    const unsigned long usec = 5000; // usec
    unsigned long loop_time = micros() - loop_start;
    if (loop_time < usec){
        delayMicroseconds(usec - loop_time);
    }
}
