
#include "imu.hpp"
// #include "pres_temp.hpp"
// #include "motors.hpp"
#include "blinkled.hpp"
#include <Wire.h>
#include <cstdint>
#include <yivo.hpp>
#include <gciSensors.hpp>
// #include <TFmini.h>

gciLSOX imu; // accel, gyro, mag, press, temp
// QuadESC motors(1000);
// BlinkLED blinker(500);
Yivo<128> yivo; // uC to computer packetizer
Alarm hertz(100);

// Earth::WGS84_t wgs;

/*
Toggles between true and false
*/
class Toggle {
  public:
  Toggle(bool v=false): val(v) {}

  bool toggle() {
    val = !val;
    return val;
  }

  void set(bool v) {val = v;}

  explicit operator bool() const { return val; }

  protected:
  bool val;
};

void status(const String& s, const bool condition) {
  String stat = condition ? "GOOD" : "FAILED";
  Serial.println(s + stat);
}

void setup() {

    Serial.begin(1000000);
    Serial.setTimeout(5);
    while (!Serial)
        delay(10);

    // i2c, fast mode
    Wire.begin();
    Wire.setClock(400000);

    // setup sensors
    imu.init();

    // Board names defined:
    // ~/Library/Arduino15/packages/*/hardware/*/*/boards.txt
    #if defined(ADAFRUIT_QTPY_M0)
    Serial.print("ADAFRUIT_QTPY_M0");
    #elif defined(ADAFRUIT_TRINKET_M0)
    Serial.print("ADAFRUIT_TRINKET_M0");
    #elif defined(ADAFRUIT_ITSYBITSY_M0_EXPRESS)
    Serial.print("ADAFRUIT_ITSYBITSY_M0_EXPRESS");
    #elif defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS)
    Serial.print("ADAFRUIT_ITSYBITSY_M4_EXPRESS");
    #else
    Serial.print("Unknown Processor");
    #endif
    Serial.println(" boot complete:");
    status(" + LSM6DSOX accel|gyro: ", imu.soxFound);
    status(" + LIS3MDL mag:         ", imu.lisFound);
    status(" + BMP390 press|temp:   ", imu.bmpFound);    
}

Toggle telemetry;

// void sendTelemetry (bool now=false) {
//   if (hertz.check() || now) {
//     if (imu.found) {
//         imu.read();
//         if (telemetry || now) yivo.pack_n_send(imu.id, imu.data.b, imu.bsize);
//     }
//   }
// }

void loop() {
  // sendTelemetry();
  if (imu.found) {
    imu.read();
    if (telemetry) 
      if (hertz.check()) yivo.pack_n_send(imu.id, imu.data.b, imu.bsize);
  }

  // serial ascii input
  if (Serial.available() > 0) {
    int inByte = Serial.read();

    if (inByte == '\n' or inByte == '\r'); // get rid of \r and \n from Master
    else if (inByte == 'g') yivo.pack_n_send(PING, nullptr, 0);
    else if (inByte == 't') telemetry.toggle();
    else if (inByte == 'T') yivo.pack_n_send(imu.id, imu.data.b, imu.bsize);
    else yivo.pack_n_send(YIVO_ERROR); // send error, unknown command
  }

  // blinker.update();
}
