#pragma once

// #include <Servo.h>
// #include "blinkled.hpp"
// // #include "packer.hpp"
// #include <messages.hpp>

// /**
// BLHeli 15a ESC
// Features from pdf manual
// - Perfect for 1806, 2204, 2205, brushless motor in QAV180, QAV 210, QAV250 and
// other small drone
// - Works with 2S to 4S input ‐ Rated at 15A continuous and 25A burst (5s max)
// - All N‐FET design with external oscillator for steady performance across
// different thermal and voltage conditions

// ## Timings

// Supports standard 1-2 msec RC servo timings at >= 50Hz

// - Minimum: 920 - 1050 usec (or 0.920 - 1.05 msec)
// - Middle: 1500 usec
// - Max 1800 - 2000 usec

// ## Power ON Sequence

// 1. Turn power on: 3 short rising tone beeps
// 2. Set middle throttle: 1 long low tone beep
// 3. Set zero throttle: 1 long high tone beep
// 4. Motor is ready to command

// */

// // ----- Motor PWM Levels -----
// constexpr int MOTOR_MAX_LEVEL  = 2000;
// constexpr int MOTOR_ZERO_LEVEL = 1000;
// constexpr int MOTOR_ARM_START  = 1500;

// // ----- Motor locations -----
// constexpr int PIN_MOTOR0 = 7;
// constexpr int PIN_MOTOR1 = 9;
// constexpr int PIN_MOTOR2 = 10;
// constexpr int PIN_MOTOR3 = 11;


// class QuadESC: public Alarm {
//   public:
//     QuadESC(const uint32_t delaytime): 
//             Alarm(delaytime), 
//             armed(false), 
//             motor0_val(MOTOR_ZERO_LEVEL), motor1_val(MOTOR_ZERO_LEVEL), 
//             motor2_val(MOTOR_ZERO_LEVEL), motor3_val(MOTOR_ZERO_LEVEL) {
//         motor0.attach(PIN_MOTOR0, 1000, 2000);
//         motor1.attach(PIN_MOTOR1, 1000, 2000);
//         motor2.attach(PIN_MOTOR2, 1000, 2000);
//         motor3.attach(PIN_MOTOR3, 1000, 2000);
//     }

//     ~QuadESC() {
//         if (armed)
//             stop();

//         motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
//         motor0.detach();
//         motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
//         motor1.detach();
//         motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
//         motor2.detach();
//         motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
//         motor3.detach();
//     }

//     // void stop() {
//     //     motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
//     //     motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
//     //     motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
//     //     motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
//     //     motor0.detach();
//     //     motor1.detach();
//     //     motor2.detach();
//     //     motor3.detach();
//     // }

//     void stop() {
//       if (!armed) return;

//       bool state[4]{true,true,true,true};
//       const float steps = 1.0 / 5.0;
//       const int incrs[4]{
//         int((motor0_val - MOTOR_ZERO_LEVEL) * steps),
//         int((motor1_val - MOTOR_ZERO_LEVEL) * steps),
//         int((motor2_val - MOTOR_ZERO_LEVEL) * steps),
//         int((motor3_val - MOTOR_ZERO_LEVEL) * steps),
//       };
//       int vals[4];

//       // int incr = (motor_val - MOTOR_ZERO_LEVEL) / 5.0;
//       // Serial.println(incr);
//       while (true){
//         int *motor_val = nullptr;
//         for (int num = 0; num < 4; ++num) {
//           switch (num) {
//             case 0:
//               motor_val = &motor0_val; break;
//             case 1:
//               motor_val = &motor1_val; break;
//             case 2:
//               motor_val = &motor2_val; break;
//             case 3:
//               motor_val = &motor3_val; break;
//           }

//           if (*motor_val > MOTOR_ZERO_LEVEL) {
//             *motor_val = motor_limit(*motor_val - incrs[num]);
//             vals[num] = *motor_val;
//           }
//           else {
//             vals[num] = MOTOR_ZERO_LEVEL;
//             state[num] = false;
//           }
//         }
//         set(vals[0],vals[1],vals[2],vals[3]);
//         // Serial.println(motor_val[0]);
//         delay(100);

//         if (state[0] == false && state[1] == false && state[2] == false && state[3] == false) break;
//       }
//     }

//     // move to constructor?
//     // void init() {
//     //     // motor0.attach(PIN_MOTOR0, 1000, 2000);
//     //     // motor1.attach(PIN_MOTOR1, 1000, 2000);
//     //     // motor2.attach(PIN_MOTOR2, 1000, 2000);
//     //     // motor3.attach(PIN_MOTOR3, 1000, 2000);
//     // }

//     /* Arms the ESC's and makes them ready for flight */
//     void arm() {
//       motor0.writeMicroseconds(MOTOR_ARM_START);
//       motor1.writeMicroseconds(MOTOR_ARM_START);
//       motor2.writeMicroseconds(MOTOR_ARM_START);
//       motor3.writeMicroseconds(MOTOR_ARM_START);

//       delay(2000);
      
//       motor0.writeMicroseconds(MOTOR_ZERO_LEVEL);
//       motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
//       motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
//       motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
      
//       delay(1000);

//       this->armed = true;
//     }

//     /* Set the ESC for each motor to a PWM */
//     void set(const int m0, const int m1, const int m2, const int m3) {
//       if (!armed) return;
//         motor0_val = motor_limit(m0);
//         motor1_val = motor_limit(m1);
//         motor2_val = motor_limit(m2);
//         motor3_val = motor_limit(m3);


//         motor0.writeMicroseconds(motor0_val);
//         motor1.writeMicroseconds(motor1_val);
//         motor2.writeMicroseconds(motor2_val);
//         motor3.writeMicroseconds(motor3_val);
//     }

//     void incr(const int delta) {
//       if (!armed) return;
//         motor0_val += delta;
//         motor1_val += delta;
//         motor2_val += delta;
//         motor3_val += delta;
//         set(motor0_val,motor1_val,motor2_val,motor3_val);
//     }

//   void ramp() {
//       if (!armed) return;

//       // ramp up to max
//       for (int i=MOTOR_ZERO_LEVEL; i < MOTOR_MAX_LEVEL; i+=100){
//         int val = i;
//         this->set(val,val,val,val);
//         delay(500);
//       }

//       // ramp down to min
//       for (int i=MOTOR_MAX_LEVEL; i > MOTOR_ZERO_LEVEL; i-=100){
//         int val = i;
//         this->set(val,val,val,val);
//         delay(500);
//       }

//       // set to 0
//       int val = MOTOR_ZERO_LEVEL;
//       this->set(val,val,val,val);
//   }

//   // void update(){
//   //   if (check()) {
//   //     packetMotor(
//   //       motor0_val,
//   //       motor1_val,
//   //       motor2_val,
//   //       motor3_val,
//   //       (armed) ? 1 : 0
//   //     );
//   //   }
//   // }

//   Motors4_t get_msg() {
//     Motors4_t m;
//     m.m0 = motor0_val;
//     m.m1 = motor1_val;
//     m.m2 = motor2_val;
//     m.m3 = motor3_val;
//     m.armed = (armed) ? 1 : 0;
//     // uint16_t msg[5]{
//     //   m0,m1,m2,m3,
//     //   (armed) ? 1 : 0
//     // };

//     return m;
//   }

//   protected:
//     Servo motor0;
//     Servo motor1;
//     Servo motor2;
//     Servo motor3;

//     int motor0_val;
//     int motor1_val;
//     int motor2_val;
//     int motor3_val;

//     bool armed;

//     inline int motor_limit(const int val){
//         // return val >= MOTOR_MAX_LEVEL ? MOTOR_MAX_LEVEL : val <= MOTOR_ZERO_LEVEL ? MOTOR_ZERO_LEVEL : val;
//         return constrain(val, MOTOR_ZERO_LEVEL, MOTOR_MAX_LEVEL);
//     }
// };