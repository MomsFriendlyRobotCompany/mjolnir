// MIT
// Kevin Walchko (c) 2020
// -----------------------------------------------------------
// accel/gyro: https://adafruit.github.io/Adafruit_LSM6DS/html/class_adafruit___l_s_m6_d_s.html
// mag: https://adafruit.github.io/Adafruit_LIS3MDL/html/class_adafruit___l_i_s3_m_d_l.html
// pressure[lps22]: https://adafruit.github.io/Adafruit_LPS2X/html/_adafruit___l_p_s2_x_8h.html
// pressure[bpm390]: https://github.com/adafruit/Adafruit_BMP3XX/blob/master/bmp3_defs.h
// conversions: https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
// TFmini: https://github.com/hideakitai/TFmini
// -----------------------------------------------------------

/*
maybe do it by boards I have:

START  0xFF
soxlis 0xF1 accel / gyro / mag / temp
s33lis 0xF0 accel / gyro / mag / temp
...
END    0xEE
sox    0xE1 accel / gyro / temp
s33    0xE0 accel / gyro / temp
...
lis    0xD0 mag
...
lidar  0xC0 dist
...
lps22  0xB2 pres / temp
dps310 0xB1 pres / temp
bmp390 0xB0 pres / temp
...
b085   0xA1 imu
b055   0xA0 imu
*/



#include <Wire.h>
#include "Arduino.h"
#include "TFmini.h"
#include "packer.hpp"
#include "sensor.hpp"
#include "rate.hpp"

// sensors
TFmini tfmini;  // distance
gciLSOXLIS imu; // accel / gyro / mag
gciDPS310 pres; // pressure / temperature

void setup(void) {
    Serial.begin(1000000); // 1Mbps
    Serial.setTimeout(1);
    while (!Serial)
        delay(1000);

    // lidar
    Serial1.begin(TFmini::DEFAULT_BAUDRATE);
    tfmini.attach(Serial1);

    Wire.setClock(400000); // 400 kHz

    // accel / gyro / mag
    imu.init();

    // pressure / temperature
    pres.init();
}

Packer packer(&Serial);
//LoopRate looprate;
float distance = 0.0f;
int cnt = 0;

void loop() {
    int numChar = Serial.available();

    if (numChar > 0){
        int b = Serial.read();

//        // Enable displaying loop rate
//        if (b == 'h'){
//            looprate.toggle();
//        }
//        // update or print debugging loop rate info
//        looprate.update();
        
        if (b == 'g') {
            imu.read();
            pres.read();
//            if (tfmini.available()) distance = tfmini.getDistance();
            if (cnt % 2 == 0 && tfmini.available()) distance = tfmini.getDistance();
            ++cnt;
        
            // Send data
            // [FF,FF,msgLen, ...., EE,EE]
            packer.begin();
            if (imu.found) packer.frame(imu.id, imu.data.b, imu.bsize); // 10
            if (pres.found) packer.frame(pres.id, pres.data.b, pres.bsize); // 2
            packer.frame(0xF7, distance);   // 1
            packer.end();
        }
    }
}
