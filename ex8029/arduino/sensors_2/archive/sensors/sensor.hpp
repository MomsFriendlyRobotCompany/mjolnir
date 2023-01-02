#pragma once

#include <stdint.h>
// #include <yivo.hpp>

template<uint32_t size>
struct Buffer {
  union {
    float    f[size];   // float
    uint32_t l[size];   // long
    uint16_t s[size*2]; // short
    uint8_t  b[size*4]; // byte
  };

  void clear() {
    memset(b, 0, size);
  }
};

template<int flsz>
class Sensor {
  public:
    Sensor(uint8_t Id): id(Id), bsize(flsz * sizeof(float)), found(false) {}
          
    bool found;
    Buffer<flsz> data;
    const uint16_t bsize; // length of array
    const uint8_t id;
  // protected:
};