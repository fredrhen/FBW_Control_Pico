//
// Created by Freds on 2021-02-07.
//

#ifndef FBW_CONTROL_PICO_ATTITUDE_H
#define FBW_CONTROL_PICO_ATTITUDE_H

#include "../Quaternion/Quaternion.h"
#include <stdio.h>
#include <math.h>
#include "../MPU9250/MPU9250.h"
#include "../Madgwick/MadgwickAHRS.h"


class Attitude {
public:

    Attitude();

    // Initilizes the Attitude by calclulating the attitude from the accel/magn
    void init(MPU9250 &IMU_obj);

    // Updates the attitude by looking both at the gyroscope and te accel/magn
    void update(MPU9250 &IMU_obj);

    // Prints the Yaw Pitch Roll to the serial in strings
    void ypr(float &yaw, float &pitch, float &roll) const;

    // Print the Yaw Pitch Roll to the serial using bytes
    //void write(Stream &serial_obj);

    // The attitude is stored in a quaternion relative to the global axis (NED)
    Quaternion<float> orientation;

private:

    //Does not read sensor there must be a IMU.readSenor() call before to get new values
    void get_buffer(MPU9250 &IMU_obj);

    // Compute the Quaternion from the accel and magn vector
    Quaternion<float> get_quat_from_accmag(const Vector<float, 3> &accel, const Vector<float, 3> &magn);

    static Quaternion<float> from_angular_rate(const Vector<float, 3> &angular_rate, int64_t dt_micros);

    // Store the IMU data in 3d vectors
    Vector<float, 3> accel; // Acceleration Vector (ms^-2)
    Vector<float, 3> gyro;  // Angular Velocity Vector (rads^-1)
    Vector<float, 3> magn;  // Magnatometer Vector (uT)

    absolute_time_t previous_time; // Time at last update
};


#endif //FBW_CONTROL_PICO_ATTITUDE_H
