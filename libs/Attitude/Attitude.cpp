//
// Created by Freds on 2021-02-07.
//

#include "pico/stdlib.h"
#include "Attitude.h"
#include "../Vector/Vector.h"


Attitude::Attitude() = default;

void Attitude::init(MPU9250 &IMU_obj) {
    get_buffer(IMU_obj);
    orientation = get_quat_from_accmag(this->accel, this->magn);

    previous_time = get_absolute_time();
}

void Attitude::update(MPU9250 &IMU_obj)
{
    get_buffer(IMU_obj);
    //printf("Got Buffer \n");

    Quaternion<float> orientation_gyro;

    Quaternion<float> orientation_mag_accel = Attitude::get_quat_from_accmag(this->accel, this->magn);

    int64_t dt_micros = absolute_time_diff_us(previous_time, get_absolute_time());
    this->previous_time = get_absolute_time();

    orientation = Madgwick::Madgwickupdate(orientation, gyro, accel, magn);
    /*
    orientation_gyro = Attitude::from_angular_rate(this->gyro, dt_micros);

    orientation_gyro = this->orientation * orientation_gyro;


    orientation = Quaternion<float>::slerp(orientation_gyro, orientation_mag_accel, 0.02);
    orientation = orientation_gyro;
     */
}

void Attitude::ypr(float &yaw, float &pitch, float &roll) const {
    orientation.as_YPR(yaw, pitch, roll);
    yaw *= 180/M_PI;
    pitch *= 180/M_PI;
    roll *= 180/M_PI;
}

void Attitude::get_buffer(MPU9250 &IMU_obj) {
    this->accel = Vector<float, 3>(IMU_obj.getAccelX_mss(), IMU_obj.getAccelY_mss(), IMU_obj.getAccelZ_mss());
    this->gyro = Vector<float, 3>(IMU_obj.getGyroX_rads(), IMU_obj.getGyroY_rads(), IMU_obj.getGyroZ_rads());
    this->magn = Vector<float, 3>(IMU_obj.getMagX_uT(), IMU_obj.getMagY_uT(), IMU_obj.getMagZ_uT());
}

Quaternion<float> Attitude::get_quat_from_accmag(const Vector<float, 3> &accel, const Vector<float, 3> &magn) {
    Vector<float, 3> down;
    Vector<float, 3> east;
    Vector<float, 3> north;

    down = -this->accel;

    east = Vector<float, 3>::cross(down, this->magn); // east = down x magn

    north = Vector<float, 3>::cross(east, down); // north = east x down

    north.normalize();
    east.normalize();
    down.normalize();

    return Quaternion<float>::from_NED(north, east, down);
}

Quaternion<float> Attitude::from_angular_rate(const Vector<float, 3> &angular_rate, int64_t dt_micros) {
    Vector<float, 3> rot_axis;
    Quaternion<float> temp;
    rot_axis = angular_rate * ((float)dt_micros * 1e-6);

    temp = Quaternion<float>(rot_axis);
    return temp;
}

