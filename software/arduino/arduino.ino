// MIT
// Kevin Walchko (c) 2020
// -----------------------------------------------------------
// accel/gyro: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s.html
// mag: https://adafruit.github.io/Adafruit_LIS3MDL/html/class_adafruit___l_i_s3_m_d_l.html
// pressure: https://adafruit.github.io/Adafruit_LPS2X/html/_adafruit___l_p_s2_x_8h.html
// -----------------------------------------------------------
#include <Wire.h>
#include "filter.h"
#include "soxsetup.h"

#define SEALEVELPRESSURE_HPA (1013.25)

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
sensors_event_t temp, tmp;
sensors_event_t mag;
//sensors_event_t pres;

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

      if(1) { // debug
            // Yaw, Pitch, Roll
            if (0) {
                compass.update(
                a.acceleration.x, a.acceleration.y, a.acceleration.z,
                mag.magnetic.x,  mag.magnetic.y, mag.magnetic.z);
                
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
                Serial.print(a.acceleration.x, 3);
                Serial.print(", ");
                Serial.print(a.acceleration.y, 3);
                Serial.print(", ");
                Serial.print(a.acceleration.z, 3);
                Serial.println(" m/s/s");
            }

            if (1) {
                Serial.print(g.gyro.x, 3);
                Serial.print(", ");
                Serial.print(g.gyro.y, 3);
                Serial.print(", ");
                Serial.print(g.gyro.z, 3);
                Serial.println(" rad/s");
            }

            if (1) {
                Serial.print(mag.magnetic.x, 3);
                Serial.print(", ");
                Serial.print(mag.magnetic.y, 3);
                Serial.print(", ");
                Serial.print(mag.magnetic.z, 3);
                Serial.println(" uT");
            }

            if (1) {
                Serial.print(bmp.pressure,2);
                Serial.print("Pa, ");
                Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA),3);
                Serial.print("m, ");
                Serial.print(bmp.temperature*9.0/5.0+32.0,2);
                Serial.println("F");
            }
      }
      else {
          m[0] = a.acceleration.x; // m/sec^2
          m[1] = a.acceleration.y;
          m[2] = a.acceleration.z;
    
          m[3] = g.gyro.x; // rads/sec
          m[4] = g.gyro.y;
          m[5] = g.gyro.z;
    
          m[6] = mag.magnetic.x;  // uT
          m[7] = mag.magnetic.y;
          m[8] = mag.magnetic.z;

          m[9] = bmp.pressure; // Pa
          m[10] = bmp.temperature; // C
    
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
