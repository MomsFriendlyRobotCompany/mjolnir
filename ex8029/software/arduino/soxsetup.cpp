#include "soxsetup.h"

extern Adafruit_LIS3MDL lis3mdl; // magnetometer
//extern Adafruit_LPS22 lps;       // pressure/temperature
//extern Adafruit_BMP3XX bmp;
extern Adafruit_LSM6DSOX sox;    // accels/gyros

const lsm6ds_data_rate_t accel_data_rate = LSM6DS_RATE_104_HZ;
const lsm6ds_accel_range_t accel_range   = LSM6DS_ACCEL_RANGE_2_G;
const lsm6ds_data_rate_t gyro_data_rate  = LSM6DS_RATE_208_HZ;
const lsm6ds_gyro_range_t gyro_range     = LSM6DS_GYRO_RANGE_2000_DPS;
const lis3mdl_dataRate_t mag_data_rate   = LIS3MDL_DATARATE_40_HZ;
//const lps22_rate_t pressure_data_rate    = LPS22_RATE_10_HZ;


void printIMU(){
  Serial.print("Accelerometer range set to: ");
    switch (sox.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  Serial.print("Accelerometer data rate set to: ");
  switch (sox.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.print("Gyro range set to: ");
  switch (sox.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    break; // unsupported range for the DSOX
  }

  Serial.print("Gyro data rate set to: ");
  switch (sox.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }

  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }
}

void sox_setup(){

  Wire.setClock(400000); // 400 kHz

  if (!sox.begin_I2C()) {
    while (1) {
      delay(1000);
      Serial.println("LSM6DSOX NOT Found!");
    }
  }
    // Accelerometer ------------------------------------------
    sox.setAccelRange(accel_range);
    sox.setAccelDataRate(accel_data_rate);

    // Gyros ----------------------------------------------------
    sox.setGyroRange(gyro_range);
    sox.setGyroDataRate(gyro_data_rate);

    // Magnetometer -----------------------------------------------------
//  if (!lis3mdl.begin_I2C()) {
//    while (1) {
//        delay(1000);
//        Serial.println("LIS3MDL NOT Found!");
//    }
//  }
//  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
//  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
//  lis3mdl.setDataRate(mag_data_rate);
//  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
//  lis3mdl.setIntThreshold(500);
//  lis3mdl.configInterrupt(false, false, true, // enable z axis
//                          true, // polarity
//                          false, // don't latch
//                          true); // enabled!

  // Pressure/Temperature -----------------------------------------------
//  if (!bmp.begin_I2C()) {
//    while (1){
//        Serial.println("BMP3 NOT Found!");
//        delay(1000);
//    }
//  }
//  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
//  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X); // sampling 90Hz
//  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
//  bmp.setOutputDataRate(BMP3_ODR_200_HZ);

//  printIMU();
}
