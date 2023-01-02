#pragma once

#include "imu.hpp"

extern gciLSOXLIS imu;
// extern gciDPS310 press;

void printAccel() {
    Serial.print("x:");
    Serial.print(imu.data.f[0]);
    Serial.print(",");

    Serial.print("y:");
    Serial.print(imu.data.f[1]);
    Serial.print(",");

    Serial.print("z:");
    Serial.println(imu.data.f[2]);
}

void printGyro() {
    Serial.print("x:");
    Serial.print(imu.data.f[3]);
    Serial.print(",");

    Serial.print("y:");
    Serial.print(imu.data.f[4]);
    Serial.print(",");

    Serial.print("z:");
    Serial.println(imu.data.f[5]);
}

void printMag() {
    Serial.print("x:");
    Serial.print(imu.data.f[6]);
    Serial.print(",");

    Serial.print("y:");
    Serial.print(imu.data.f[7]);
    Serial.print(",");

    Serial.print("z:");
    Serial.println(imu.data.f[8]);
}

void printQuaternion() {
    Serial.print("w:");
    Serial.print(imu.data.f[10]);
    Serial.print(",");

    Serial.print("x:");
    Serial.print(imu.data.f[11]);
    Serial.print(",");

    Serial.print("y:");
    Serial.print(imu.data.f[12]);
    Serial.print(",");

    Serial.print("z:");
    Serial.println(imu.data.f[13]);
}