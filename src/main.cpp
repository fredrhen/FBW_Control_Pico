
#include <stdio.h>
//#include <string.h>
#include "pico/stdlib.h"
#include "../libs/MPU9250/MPU9250.h"
//#include "../libs/Vector/Vector.h"
#include "../libs/Attitude/Attitude.h"

#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"

#define freq 500.f

MPU9250 IMU(5);
Attitude my_atti;

int main() {
    stdio_init_all();
    int i = IMU.begin();
    IMU.readSensor();
    my_atti.init(IMU);

    IMU.setAccelCalX(-8.181, 1.00);
    IMU.setAccelCalY(-4.952, .998);
    IMU.setAccelCalZ(-0.294, 1.000);

    IMU.setMagCalX(-30.09, 1.027);
    IMU.setMagCalY(-18.0, .903);
    IMU.setMagCalZ(-6.53, 1.08);

    int status = IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    if (status != 1)
    {
        printf("failed \n");
    }


    absolute_time_t from, to, last;
    printf("Setup Successful \n");
    float yaw, pitch, roll;
    float ayaw, apitch, aroll;

    float cyc_f;

    uint64_t micro_sec =(uint64_t)((1/ freq) * 1e6);

    last = get_absolute_time();
    while (1) {
        from = get_absolute_time();
        IMU.readSensor();
        my_atti.update(IMU);

        cyc_f = 1 / (float)(absolute_time_diff_us(last, get_absolute_time())) * 1e6;
        last = get_absolute_time();

        my_atti.ypr(yaw, pitch, roll);



        printf("Yaw: %.7f, Pitch: %.7f, Roll: %.7f, freq : %.0f \n", yaw, pitch, roll, cyc_f);

        //printf("[ %f, %f, %f, %f] \n", my_atti.orientation._x, my_atti.orientation._u, my_atti.orientation._v, my_atti.orientation._w);
        to = get_absolute_time();
        while(absolute_time_diff_us(from, to) < micro_sec)
        {
            sleep_us(5);
            to = get_absolute_time();
        }
        //printf("Micro Sec: %lli , Frequency: %f \n", micro_sec, freq);

    }

    return 0;
}

#pragma clang diagnostic pop