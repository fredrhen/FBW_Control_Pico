//
// Created by Freds on 2021-02-07.
//

#include <stdio.h>
//#include <string.h>
#include "pico/stdlib.h"
#include "../libs/MPU9250/MPU9250.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

MPU9250 IMU(5);


int main() {
    stdio_init_all();
    int i = IMU.begin();
    IMU.readSensor();




    sleep_ms(1000);
    printf("Starting Magnetometer Calibration, Move it in a figure 8 \n");
    int status = IMU.calibrateMag();

    (status >0) ? printf("Success \n") : printf("Calibration Failed\n");
    printf("Mag Bias (uT): X: %f, Y: %f, Z: %f \n", IMU.getMagBiasX_uT(), IMU.getMagBiasY_uT(), IMU.getMagBiasZ_uT());
    printf("Mag Scale (): X: %f, Y: %f, Z: %f \n", IMU.getMagScaleFactorX(), IMU.getMagScaleFactorY(), IMU.getMagScaleFactorZ());

    printf("Setup Successful \n");

    return 0;
}

#pragma clang diagnostic pop