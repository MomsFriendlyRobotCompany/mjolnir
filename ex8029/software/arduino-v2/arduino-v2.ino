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
#include "filter.h"
#include "soxsetup.h"
#include <Adafruit_Sensor.h> // SENSORS_PRESSURE_SEALEVELHPA, SENSORS_GRAVITY_STANDARD

// #define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_LIS3MDL lis3mdl; // magnetometer
Adafruit_BMP3XX bmp;
Adafruit_LSM6DSOX sox;    // accels/gyros

//Madgwick mad(1.0);
//eCompass compass;


//unsigned int cnt=0;
 float m[11]; // // accel(3), gyro(3), mag(3), pressure, temperature
//float m[8]; // accel(3), gyro(3), pressure, temperature
byte const* p = reinterpret_cast<byte const *>(m);
sensors_event_t a;
sensors_event_t g;
sensors_event_t tmp;
sensors_event_t mag;
//sensors_event_t pres;
float ax, ay, az;
float mx, my, mz;
float wx, wy, wz;
float pres, alt;
float temp;
//float roll, pitch, yaw;
unsigned long start = 0;
unsigned int count = 1;

/*
 * trying to reduce comm on bus, reading sensors only when
 * it makes sense.
1/200 = 0.005 = 5 ms
accel 208  200   1
gyros 208  200   1
mags   40   40   5
pres   10   10   20
*/

void setup(void) {
    Serial.begin(115200);
    Serial.setTimeout(1);
    while (!Serial)
        delay(100); // will pause Zero, Leonardo, etc until serial console opens
    
    Wire.setClock(400000); // 400 kHz

  if (!sox.begin_I2C()) {
    while (1) {
      delay(100);
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
            delay(100);
            Serial.println("LIS3MDL NOT Found!");
        }
      }
//      lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE); // 155 already does this
//      lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE); // 300 already does this
      lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
      lis3mdl.setDataRate(LIS3MDL_DATARATE_300_HZ);
      lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
//      lis3mdl.setIntThreshold(500);
//      lis3mdl.configInterrupt(false, false, true, // enable z axis
//                              true, // polarity
//                              false, // don't latch
//                              true); // enabled!

    // Pressure/Temperature -----------------------------------------------
    if (!bmp.begin_I2C()) {
        while (1){
            Serial.println("BMP3 NOT Found!");
            delay(100);
        }
    }
//    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
//    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X); // sampling 90Hz
//    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_31); // 1,3,7,15,31

    // drone, table 9, pg 17
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  start = millis();

}

void loop() {
    bool freeRun = false;
    int b = 0;
//
//    if (Serial.available() > 0) {
//        b = Serial.read();
//    }
    
    // not sure this helps, Adafruit_LSM6DS::_read() always gets
    // accel/gyros ... does it block?
    sox.getEvent(&a,&g,&tmp);
    lis3mdl.getEvent(&mag);
//    bmp.performReading();
//    if (!bmp.performReading()) Serial.println("ERROR: BMP");

    ax = a.acceleration.x / SENSORS_GRAVITY_STANDARD;
    ay = a.acceleration.y / SENSORS_GRAVITY_STANDARD;
    az = a.acceleration.z / SENSORS_GRAVITY_STANDARD;
    
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

    if (1 && (count % 100 == 0)){
//        Serial.println(" ");
        unsigned long now = millis();
        float hz = 1000.0*float(count+1)/float(now - start);
        Serial.println(hz,2);
//        Serial.println(pres,2);
//        Serial.println(" ");
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
}
