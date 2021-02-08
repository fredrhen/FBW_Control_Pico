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

    IMU.setAccelCalX(-8.181, 1.00);
    IMU.setAccelCalY(-4.952, .998);
    IMU.setAccelCalZ(-0.294, 1.000);


    IMU.readSensor();

    float ax, ay, az, mag;

    while(1){
        IMU.readSensor();
        ax = IMU.getAccelX_mss();
        ay = IMU.getAccelY_mss();
        az = IMU.getAccelZ_mss();

        mag = ax * ax + ay * ay + az * az;
        mag = sqrt(mag);

        //printf(" Mag : %.6f %.6f %.6f \n", IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
        printf(" %.6f %.6f %.6f %.6f \n", ax, ay, az, mag);
        //printf(" Gyr : %.6f %.6f %.6f \n \n", IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads());
        sleep_ms(100);
    }

    return 0;
}

#pragma clang diagnostic pop

