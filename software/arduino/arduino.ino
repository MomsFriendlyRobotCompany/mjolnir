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


Adafruit_LIS3MDL lis3mdl; // magnetometer
Adafruit_LPS22 lps;       // pressure/temperature
Adafruit_LSM6DSOX sox;    // accels/gyros

//Madgwick mad(1.0);
eCompass compass;


unsigned int cnt=0;
float m[11];
sensors_event_t a;
sensors_event_t g;
sensors_event_t temp, tmp;
sensors_event_t mag;
sensors_event_t pres;

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
//  byte *p = (byte*)m;
  byte const* p = reinterpret_cast<byte const *>(m);

  if (Serial.available() > 0) {
    // read the incoming byte:
    char b = Serial.read();

    if (b == 'g'){
//    if (1){
        // not sure this helps, Adafruit_LSM6DS::_read() always gets
        // accel/gyros ... does it block?
      sox.getEvent(&a,&g,&tmp);
      if (cnt%5 == 0) lis3mdl.getEvent(&mag);
      if (cnt%20 == 0) lps.getEvent(&pres, &temp); // get pressure/temp


      if(0) { // debug
        
        compass.update(
            a.acceleration.x, a.acceleration.y, a.acceleration.z,
            mag.magnetic.x,  mag.magnetic.y, mag.magnetic.z);

        compass.getEuler(roll, pitch, yaw);
      
        // Yaw, Pitch, Roll
//        Serial.print("Orientation: ");
        Serial.print(yaw, 2);
        Serial.print(", ");
        Serial.print(pitch, 2);
        Serial.print(", ");
        Serial.println(roll, 2);
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
    
          m[9] = pres.pressure; // hPa
    
          m[10] = temp.temperature; // C
    
          Serial.write(0xff);
          Serial.write(p, sizeof(m));
      }
    }
    else {
        Serial.println("*");
    }

  cnt += 1;
  if (cnt == 65000) cnt = 0;  // value?
  }

  delay(2); // 5ms => 200Hz | 4ms => 250Hz
}
