#pragma once

/**
Take a look at borderflight drivers:

lis3mdl: https://github.com/bolderflight/lis3mdl
bmp3: https://github.com/bolderflight/bmp3
lsm6dsox: None

Another:
lsm6ssox: https://github.com/arduino-libraries/Arduino_LSM6DSOX

*/

#include <stdint.h>
#include <cmath>

template<uint32_t size> // number of floats
struct Buffer {
  union {
    float    f[size];   // float
    uint32_t l[size];   // long
    uint16_t s[size*2]; // short
    uint8_t  b[size*4]; // byte
  };

  void clear() {
    memset(b, 0, size<<2); // changed this, move bsize to Buffer class?
  }
};


template<uint32_t flsz> // number of floats
class mSensor {
  public:
    mSensor(uint8_t Id): id(Id), bsize(flsz * 4), found(false) {}
    
    bool found;
    Buffer<flsz> data;
    const uint16_t bsize; // length of array
    const uint8_t id;
};
