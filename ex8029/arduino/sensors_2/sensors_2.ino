
#include "imu.hpp"
// #include "pres_temp.hpp"
// #include "motors.hpp"
#include "blinkled.hpp"
#include <Wire.h>
#include <cstdint>
#include <yivo.hpp>
#include <gciSensors.hpp>
#include <TFmini.h>

gciLSOX imu; // accel, gyro, mag, press, temp
// QuadESC motors(1000);
BlinkLED blinker(500);
Yivo<128> yivo; // uC to computer packetizer
Alarm hertz(5);

// Earth::WGS84_t wgs;

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

    Serial.println("Boot complete:");
    Serial.println(" " + imu.found ? "+ IMU ready" : "! IMU not found");

    Serial.println(" Motors ready");
    
}

Toggle telemetry;

void sendTelemetry (bool now=false) {
  if (hertz.check() || now) {
    if (imu.found) {
        imu.read();
        if (telemetry || now) yivo.pack_n_send(imu.id, imu.data.b, imu.bsize);
    }
  }
}

void loop() {
  sendTelemetry();

  // serial ascii input
  if (Serial.available() > 0) {
    int inByte = Serial.read();

    if (inByte == '\n' or inByte == '\r'); // get rid of \r and \n from Master
    else if (inByte == 'g') yivo.pack_n_send(PING, nullptr, 0);
    else if (inByte == 't') telemetry.toggle(); //telemetry = !telemetry;
    else if (inByte == 'T') sendTelemetry(true);
    else yivo.pack_n_send(YIVO_ERROR); // send error, unknown command
  }

  blinker.update();
}
