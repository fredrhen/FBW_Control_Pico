//
// Created by Freds on 2021-02-07.
//

//
// Created by Freds on 2021-02-07.
//

#include <stdio.h>
//#include <string.h>
#include "pico/stdlib.h"
#include "../libs/MPU9250/MPU9250.h"
#include "math.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

MPU9250 IMU(5);


int main() {
    stdio_init_all();
    int i = IMU.begin();

    IMU.readSensor();

    IMU.setAccelCalX(-8.181, 1.00);
    IMU.setAccelCalY(-4.952, .998);
    IMU.setAccelCalZ(-0.294, 1.000);

    float ax, ay, az, mag;

    while(1){
        IMU.readSensor();
        ax = IMU.getAccelX_mss();
        ay = IMU.getAccelY_mss();
        az = IMU.getAccelZ_mss();


        mag = ax * ax + ay * ay + az * az;
        mag = sqrt(mag);

        printf("%.6f %.6f %.6f %.6f \n", ax, ay, az, mag);
        sleep_ms(10);
    }

    return 0;
}

#pragma clang diagnostic pop

