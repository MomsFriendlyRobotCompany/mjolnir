#pragma once

// #include <cstdint>
// #include <cstring> // memset

/*
message format

[0xff,0xff, msg, size, ... , time]

messages:
- imu,   100hz, 56B,  44800bps,
- motors, 10hz,  4B,    320bps, 4 bytes, 0-180 deg, 0.7 deg
- battery, 1hz,  4B,     32bps, 1 int
- usonic, 10hz, 16B,   1280bps, 4 ints

Msg  Sensor  Size[F/B]
-------------------
0xD0 IMU      15/60
0xD1 usonic    4/16
0xD2 battery   1/4
*/
// constexpr int maxMsgSize = 15 * sizeof(float) + 5;


// union {
//     byte b[maxMsgSize * sizeof(float)];
//     float f[maxMsgSize];
//     //   unsigned long l[maxMsgSize];
//     uint32_t l[maxMsgSize];
// } msgBuf;

// void pack_n_send(uint8_t msg, uint8_t size, uint8_t *buff) {
//     // if (size > maxMsgSize) return; // not sure what to do here

//     memset(msgBuf.b, 0, maxMsgSize);
//     msgBuf.b[0] = 0xFF;
//     msgBuf.b[1] = 0xFF;
//     msgBuf.b[2] = msg;
//     msgBuf.b[3] = size;
//     memcpy(&msgBuf.b[4], buff, size);

//     Serial.write(msgBuf.b, size + 4);
// }

// void packetMotor(int m0, int m1, int m2, int m3, uint8_t armed) {
//   constexpr int msgSize = 5;
//   constexpr uint8_t  MOTORS = 0xE0;
  
//   uint8_t msg[5]{
//     uint8_t(m0 >> 2),
//     uint8_t(m1 >> 2),
//     uint8_t(m2 >> 2),
//     uint8_t(m3 >> 2),
//     armed
//   };

//   pack_n_send(MOTORS, msgSize, msg);

// }