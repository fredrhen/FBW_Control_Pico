
#include <stdio.h>
//#include <string.h>
#include "pico/stdlib.h"
#include "MPU9250.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

MPU9250 IMU(0, 5);

int main() {
    stdio_init_all();
    int i = IMU.begin();
    absolute_time_t from;
    int64_t time_diff;

    while (1) {
        from = get_absolute_time();
        IMU.readSensor();
        time_diff = absolute_time_diff_us(from, get_absolute_time());

        printf("Accel. X = %f, Y = %f, Z = %f \n", IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss());
        printf("Gyro. X = %f, Y = %f, Z = %f \n", IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads());
        printf("Magn. X = %f, Y = %f, Z = %f \n", IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT());
        printf("Time taken (us): %lli", time_diff);
        printf("\n");

        sleep_ms(2000);

    }

    return 0;
}

#pragma clang diagnostic pop