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

    IMU.setMagCalX(-30.09, 1.027);
    IMU.setMagCalY(-18.0, .903);
    IMU.setMagCalZ(-6.53, 1.08);


    IMU.readSensor();



    while(1){
        IMU.readSensor();
        printf("%.6f %.6f %.6f \n", IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
        sleep_ms(100);
    }

    return 0;
}

#pragma clang diagnostic pop

