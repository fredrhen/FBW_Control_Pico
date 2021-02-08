//
// Created by Freds on 2021-02-06.
//

#ifndef FBW_CONTROL_PICO_QUATERNION_H
#define FBW_CONTROL_PICO_QUATERNION_H

#include "Quaternion.h"
#include "../Vector/Vector.h"
#include <math.h>
#include "pico/stdlib.h"

template<typename T>
class Quaternion {
public:
    Quaternion();

    Quaternion(Vector<T, 3> &rot_axis);

    Quaternion(T u, T v, T w, T x);

    void as_YPR(T &yaw, T &pitch, T &roll) const;

    void normalize();

    Quaternion conjugate() const;

    static Quaternion slerp(const Quaternion &quat0, const Quaternion &quat1, float amount);

    static Quaternion from_NED(const Vector<T, 3> &north, const Vector<T, 3> &east, const Vector<T, 3> &down);

    Quaternion operator*(const Quaternion &quat1);

    Quaternion operator*(const T scalefactor);

    Quaternion operator+(const Quaternion &quat1);

    Quaternion operator-(const Quaternion &quat1);

    Quaternion& operator=(const Quaternion &quat1);


    T _u, _v, _w, _x; // x is the real part and [u, v, w] is the imaginary part


};



#endif //FBW_CONTROL_PICO_QUATERNION_H
