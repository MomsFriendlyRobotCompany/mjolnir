#pragma once

#include <gciSensors.hpp>

constexpr int LED_PIN = 13;

// Toggle board's LED on/off
class BlinkLED: public Alarm {
  public:
  BlinkLED(const uint32_t delaytime, int pin=LED_PIN): led_blink(true), Alarm(delaytime), led_pin(pin) {}

  void update() {
    if (check()) {
      led(led_blink);
      led_blink = !led_blink;
    }
  }

  protected:

  void led(bool val) {
      if (val) digitalWrite(led_pin, HIGH);
      else digitalWrite(led_pin, LOW);
  }

  bool led_blink;
  const int led_pin;
};


//////////////////////////////////////////

// class Hertz {
//   public:
//   Hertz(const uint32_t delaytime): mark(millis()), dt(delaytime) {}

//   bool check(){
//     uint32_t now = millis();
//     if (now > mark){
//       mark = now + dt;
//       return true;
//     }
//     return false;
//   }

//   protected:
//   const uint32_t dt;
//   uint32_t mark;
// };
