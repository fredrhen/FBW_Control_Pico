#ifndef FBW_CONTROL_PICO_MADGWICKAHRS_H
#define FBW_CONTROL_PICO_MADGWICKAHRS_H

#include <stdio.h>
#include <math.h>
#include "../Vector/Vector.h"
#include "../Quaternion/Quaternion.h"

struct Madgwick {
    static float invSqrt(float x);

    static void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,
                            float q0, float q1, float q2, float q3, float *new_quat);

    static Quaternion<float>
    Madgwickupdate(const Quaternion<float> &quat, Vector<float, 3> &gyro, Vector<float, 3> &accel, Vector<float, 3> &magn);
};

#endif
