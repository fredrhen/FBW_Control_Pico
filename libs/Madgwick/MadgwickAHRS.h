//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef FBW_CONTROL_PICO_MADGWICKAHRS_H
#define FBW_CONTROL_PICO_MADGWICKAHRS_H

//----------------------------------------------------------------------------------------------------
// Variable declaration
#include "pico/stdlib.h"
#include "../Vector/Vector.h"
#include "../Quaternion/Quaternion.h"
//---------------------------------------------------------------------------------------------------
// Function declarations

struct Madgwick {
    static float invSqrt(float x);

    static void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
                            float q0, float q1, float q2, float q3, float *new_quat);

    static void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,
                               float q0, float q1, float q2, float q3, float *new_quat);

    static Quaternion<float>
    Madgwickupdate(const Quaternion<float> &quat, Vector<float, 3> gyro, Vector<float, 3> accel, Vector<float, 3> magn);
};

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
