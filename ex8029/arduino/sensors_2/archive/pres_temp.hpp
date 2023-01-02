#pragma once

// // #include <gciSensors.hpp>
// #include "sensor.hpp"

// /////////////////////////////////////////////////////////////////////////////////////
// #if 0
// class PressureSensor: public Sensor<2> {
//   public:
//   PressureSensor(): Sensor<2>(PRES_TEMP) {}

//   float altitude(const float p) {
//         // Probably best not to run here ... very computational.
//         // pre compute some of this?
//         // call atmospalt() ... like matlab?
//         // same as mean sea level (MSL) altitude
//         // Altitude from pressure:
//         // https://www.mide.com/air-pressure-at-altitude-calculator
//         // const float Tb = 15; // temperature at sea level [C] - doesn't work
//         // const float Lb = -0.0098; // lapse rate [C/m] - doesn't work ... pow?
//         constexpr float Tb = 288.15;           // temperature at sea level [K]
//         constexpr float Lb = -0.0065;          // lapse rate [K/m]
//         constexpr float Pb = 101325.0;         // pressure at sea level [Pa]
//         constexpr float R  = 8.31446261815324; // universal gas const [Nm/(mol K)]
//         constexpr float M  = 0.0289644;        // molar mass of Earth's air [kg/mol]
//         constexpr float g0 = 9.80665;          // gravitational const [m/s^2]

//         return (Tb / Lb) * (std::pow(p / Pb, -R * Lb / (g0 * M)) - 1.0);
//     }
// };
// #endif

// #if 0 // start BMP3XX ======================================================
// // #include <Adafruit_BMP3XX.h>
// // #include <GCI_BMP3XX.h>

// // constexpr int pressfloats = 2;

// class gciBMP390: public PressureSensor {
//   public:

//     gciBMP390() {}

//     // Sets up the sensor
//     void init() {
//         if (bmp.begin_I2C()) {
//             /* 
//             Confused how i can set a sample rate and oversample
//             independant of each other ... shouldn't they be coupled?

//             dps310 datasheet, pg 30
//             os - oversample
//             mt - measurement time [ms]
//             prec - precision [Pa rms]
                      
//              os     mt  prec      Hz
//             -------------------------------
//             128  206.8  0.2     4.84
//              64  104.4  0.2     9.57
//               8   14.8  0.4    67.57
//               4    8.4  0.5   119.04
//               2    5.2  1.0   192.31          
//             */ 
//             // Set up oversampling and filter initialization
//             bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
//             bmp.setPressureOversampling(BMP3_OVERSAMPLING_2X);
//             bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
//             bmp.setOutputDataRate(BMP3_ODR_100_HZ);

//             // wait until we have at least one good measurement
//             // while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
//             //     delay(10);
//             // }

//             found = true;
//         }

//         // assume startup is on the ground :P
//         // save ground measurements for reference
//         read();
//         gnd_press = data.f[1];
//         gnd_alt = altitude(gnd_press);
//     }

//     void read() {
//         if (found) {
//             // if (dps.temperatureAvailable() || dps.pressureAvailable()) {
//             //     dps.getEvents(&temp_event, &pressure_event);
//             //     data.f[0] = temp_event.temperature;
//             //     data.f[1] = pressure_event.pressure;
//             // }
//             if (bmp.performReading()) {
//               data.f[0] = bmp.temperature; // C
//               data.f[1] = bmp.pressure;    // pascals
//             }
//         }
//     }

//     // bool found;
//     // Buffer<pressfloats> data;
//     // const uint16_t bsize; // length of array
//     // const uint8_t id;

//   protected:
//     // Adafruit_BMP3XX bmp; // pressure / temperature
//     GCI_BMP3XX bmp;
//     float gnd_press;
//     float gnd_alt;
// };

// #endif // end BMP3XX =======================================================




// #if 0 // start DPS310 ==========================================================
// #include <Adafruit_DPS310.h>

// // constexpr int pressfloats = 2;

// class gciDPS310: public PressureSensor {
//   public:
//     // static const uint8_t id = uint8_t(Sensors::DPS310);

//     gciDPS310() : bsize(2 * sizeof(float)), 
//           found(false),
//           id(PRES_TEMP) {}

//     // Sets up the sensor
//     void init() {
//         if (dps.begin_I2C()) {
//             /* 
//             Confused how i can set a sample rate and oversample
//             independant of each other ... shouldn't they be coupled?

//             dps310 datasheet, pg 30
//             os - oversample
//             mt - measurement time [ms]
//             prec - precision [Pa rms]
                      
//              os     mt  prec      Hz
//             -------------------------------
//             128  206.8  0.2     4.84
//              64  104.4  0.2     9.57
//               8   14.8  0.4    67.57
//               4    8.4  0.5   119.04
//               2    5.2  1.0   192.31          
//             */ 
//             dps.configurePressure(DPS310_128HZ, DPS310_64SAMPLES);
//             dps.configureTemperature(DPS310_128HZ, DPS310_64SAMPLES);
//             dps.setMode(DPS310_CONT_PRESTEMP);

//             // wait until we have at least one good measurement
//             while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
//                 delay(10);
//             }

//             found = true;
//         }

//         // assume startup is on the ground :P
//         // save ground measurements for reference
//         read();
//         gnd_press = data.f[1];
//         gnd_alt = altitude(gnd_press);
//     }

//     void read() {
//         if (found) {
//             if (dps.temperatureAvailable() || dps.pressureAvailable()) {
//                 dps.getEvents(&temp_event, &pressure_event);
//                 data.f[0] = temp_event.temperature;
//                 data.f[1] = pressure_event.pressure;
//             }
//         }
//     }

//     bool found;
//     Buffer<2> data;
//     const uint16_t bsize; // length of array
//     const uint8_t id;

//   protected:
//     Adafruit_DPS310 dps; // pressure / temperature
//     sensors_event_t temp_event, pressure_event;
//     float gnd_press;
//     float gnd_alt;
// };

// #endif // end DPS310 =======================================================