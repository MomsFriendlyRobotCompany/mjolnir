// MIT
// Kevin Walchko (c) 2020
// -----------------------------------------------------------
// accel/gyro: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s.html
// mag: https://adafruit.github.io/Adafruit_LIS3MDL/html/class_adafruit___l_i_s3_m_d_l.html
// pressure: https://adafruit.github.io/Adafruit_LPS2X/html/_adafruit___l_p_s2_x_8h.html
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
eCompass compass;


//unsigned int cnt=0;
float m[11];
byte const* p = reinterpret_cast<byte const *>(m);
sensors_event_t a;
sensors_event_t g;
sensors_event_t tmp;
sensors_event_t mag;
//sensors_event_t pres;
float ax, ay, az;
float mx, my, mz;
float wx, wy, wz;
float pres, temp, alt;
float roll, pitch, yaw;

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
  Serial.setTimeout(10);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  sox_setup();
  
}

void loop() {
    bool freeRun = true;
    int b = 0;
    
    if (Serial.available() > 0) {
        int b = Serial.read();
    }

    if (b == 'g' || freeRun){
        // not sure this helps, Adafruit_LSM6DS::_read() always gets
        // accel/gyros ... does it block?
      sox.getEvent(&a,&g,&tmp);
      if (0) lis3mdl.getEvent(&mag);
      if (0) {
        if (!bmp.performReading()) Serial.println("ERROR: BMP");
      }
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
      // temp = tmp.temperature; // C - faster? 
      // temp = bmp.temperature; // C
      temp = bmp.temperature*9.0/5.0+32.0; // F
      alt = bmp.readAltitude(SENSORS_PRESSURE_SEALEVELHPA); // m

      if(1) { // debug
            // Yaw, Pitch, Roll
            if (0) {
                compass.update(ax, ay, az, mx,  my, mz);
                compass.getEuler(roll, pitch, yaw);
                
                Serial.print("Orientation: ");
                Serial.print(yaw, 2);
                Serial.print(", ");
                Serial.print(pitch, 2);
                Serial.print(", ");
                Serial.print(roll, 2);
                Serial.println(" deg");
            }

            if (1){
                Serial.print(ax, 3);
                Serial.print(", ");
                Serial.print(ay, 3);
                Serial.print(", ");
                Serial.print(az, 3);
                Serial.println(" m/s/s");
            }

            if (1) {
                Serial.print(wx, 3);
                Serial.print(", ");
                Serial.print(wy, 3);
                Serial.print(", ");
                Serial.print(wz, 3);
                Serial.println(" rad/s");
            }

            if (1) {
                Serial.print(mx, 3);
                Serial.print(", ");
                Serial.print(my, 3);
                Serial.print(", ");
                Serial.print(mz, 3);
                Serial.println(" uT");
            }

            if (1) {
                Serial.print(pres/100.0,2);
                Serial.print(" hPa, ");
                Serial.print(alt,3);
                Serial.print(" m, ");
                Serial.print(temp,2);
                Serial.println(" F");
            }
      }
      else {
          m[0] = ax; // m/sec^2
          m[1] = ay;
          m[2] = az;
    
          m[3] = wx; // rads/sec
          m[4] = wy;
          m[5] = wz;
    
          m[6] = mx;  // uT
          m[7] = my;
          m[8] = mz;

          m[9] = pres;  // Pa
          m[10] = temp; // C
          m[11] = alt;  // m
    
          Serial.write(0xff);
          Serial.write(p, sizeof(m));
      }
    }
    else {
        Serial.println("*");
    }

//  cnt += 1;
//  if (cnt == 65000) cnt = 0;  // value?
//  }
//  delay(100);

//  delay(2); // 5ms => 200Hz | 4ms => 250Hz
}
