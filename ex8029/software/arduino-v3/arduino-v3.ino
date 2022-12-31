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
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD

#include <Adafruit_BNO055.h>

Adafruit_LIS3MDL lis3mdl; // magnetometer
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;    // accels/gyros
Adafruit_BNO055 bno;

 float m[11]; // // accel(3), gyro(3), mag(3), pressure, temperature
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
//float roll, pitch, yaw;
unsigned long start = 0;
unsigned int count = 1;
const float invg = 1.0/SENSORS_GRAVITY_STANDARD;


void setup(void) {
    Serial.begin(115200);
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

    if (1) {
        // datasheet, drone, table 9, pg 17
        bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    }
    else {
        bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
        bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
        bmp.setOutputDataRate(BMP3_ODR_200_HZ);
    }

    bno = Adafruit_BNO055();
    if(!bno.begin()) {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while(1);
    }

    bno.setExtCrystalUse(true);

  start = millis();

}


void loop() {
    unsigned long loop_start = micros();
    bool freeRun = false;
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
    mx = mag.magnetic.z;

    pres = bmp.pressure;
    temp = tmp.temperature; // C - faster?
    // temp = bmp.temperature; // C
    // temp = bmp.temperature*9.0/5.0+32.0; // F
    // alt = bmp.readAltitude(SENSORS_PRESSURE_SEALEVELHPA); // m


    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> bnoa = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> bnow = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> bnom = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    imu::Quaternion q = bno.getQuat();

    if (1 && (count % 100 == 0)){
        unsigned long now = millis();
        float hz = 1000.0*float(count+1)/float(now - start);
        Serial.println(hz,2);
//        Serial.println(pres,2);
//        Serial.print(wy,2);
//        Serial.print(",");
//        Serial.println(wx,2);
        count = 0;
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
    if(b == 'g' || freeRun) {
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

        Serial.write(0xff);
        Serial.write(p, sizeof(m));
        Serial.write('\n');
    }

    // 5000  -> 200Hz
    // 20000 -> 50Hz
    const unsigned long usec = 5000; // usec
    unsigned long loop_time = micros() - loop_start;
    if (loop_time < usec){
        delayMicroseconds(usec - loop_time);
    }
}
