//
// Created by Freds on 2021-02-06.
//

#include "Quaternion.h"
#define EPSILON 0.0001


template<typename T>
Quaternion<T>::Quaternion() {
    this->_x = 1;
    this->_u = 0;
    this->_v = 0;
    this->_w = 0;
}

template<typename T>
Quaternion<T>::Quaternion(Vector<T, 3> &rot_axis)
{
    Vector<T, 3> temp;
    temp = rot_axis;
    T angle;

    temp.normalize(angle);

    if(angle > EPSILON )
    {
        this->_x = cos(angle/2);
        this->_u = temp[0] * sin(angle/2);
        this->_v = temp[1] * sin(angle/2);
        this->_w = temp[2] * sin(angle/2);
    }

    else
    {
        this->_x = 1;
        this->_u = 0;
        this->_v = 0;
        this->_w = 0;
    }


}

template<typename T>
Quaternion<T>::Quaternion(T argu, T argv, T argw, T argx)
        : _u(argu), _v(argv), _w(argw), _x(argx)
{
}

template<typename T>
void Quaternion<T>::as_YPR(T &yaw, T &pitch, T &roll) const {
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (this->_x * this->_u + this->_v * this->_w);
    float cosr_cosp = 1 - 2 * (this->_u * this->_u + this->_v * this->_v);
    roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2 * (this->_x * this->_v - this->_w * this->_u);
    if (abs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (this->_x * this->_w + this->_u * this->_v);
    float cosy_cosp = 1 - 2 * (this->_v * this->_v + this->_w * this->_w);
    yaw = atan2(siny_cosp, cosy_cosp);
}

template<typename T>
void Quaternion<T>::normalize() {
    T num2 = (((this->_u * this->_u) + (this->_v * this->_v)) + (this->_w * this->_w)) + (this->_x * this->_x);
    T num = 1 / sqrt(num2);
    this->_u *= num;
    this->_v *= num;
    this->_w *= num;
    this->_x *= num;
}

template<typename T>
Quaternion<T> Quaternion<T>::conjugate() const {
    return Quaternion<T>(-this->_u, -this->_v, -this->_w, this->_x);
}

template<typename T>
Quaternion<T> Quaternion<T>::slerp(const Quaternion &quat0, const Quaternion &quat1, float amount) {
    T num2;
    T num3;
    T u, v, w, x;
    T num = amount;
    T num4 = (((quat0._u * quat1._u) + (quat0._v * quat1._v)) + (quat0._w * quat1._w)) + (quat0._x * quat1._x);
    bool flag = false;
    if (num4 < 0.0)
    {
        flag = true;
        num4 = -num4;
    }
    if (num4 > 0.999999)
    {
        num3 = 1.0 - num;
        num2 = flag ? -num : num;
    }
    else
    {
        T num5 = acos(num4);
        T num6 = (1.0 / sin(num5));
        num3 = (sin(((1.0 - num) * num5))) * num6;
        num2 = flag ? ((-sin(num * num5)) * num6) : ((sin((num * num5))) * num6);
    }
    w = (num3 * quat0._w) + (num2 * quat1._w);
    u = (num3 * quat0._u) + (num2 * quat1._u);
    v = (num3 * quat0._v) + (num2 * quat1._v);
    x = (num3 * quat0._x) + (num2 * quat1._x);
    return Quaternion<T>(u, v, w, x);
}

template<typename T>
Quaternion<T> Quaternion<T>::from_NED(const Vector<T, 3> &north, const Vector<T, 3> &east, const Vector<T, 3> &down) {
    T m00 = north[0];
    T m01 = north[1];
    T m02 = north[2];

    T m10 = east[0];
    T m11 = east[1];
    T m12 = east[2];

    T m20 = down[0];
    T m21 = down[1];
    T m22 = down[2];

    T tr = m00 + m11 + m22;

    T x, u, v, w;

    if (tr > 0) {
        T S = sqrt(tr+1.0) * 2; // S=4*qw
        x = 0.25 * S;
        u = (m21 - m12) / S;
        v = (m02 - m20) / S;
        w = (m10 - m01) / S;
    }
    else if ((m00 > m11)&(m00 > m22)) {
        T S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx
        x = (m21 - m12) / S;
        u = 0.25 * S;
        v = (m01 + m10) / S;
        w = (m02 + m20) / S;
    }
    else if (m11 > m22) {
        T S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
        x = (m02 - m20) / S;
        u = (m01 + m10) / S;
        v = 0.25 * S;
        w = (m12 + m21) / S;
    }
    else {
        T S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
        x = (m10 - m01) / S;
        u = (m02 + m20) / S;
        v = (m12 + m21) / S;
        w = 0.25 * S;
    }


    return Quaternion<T>(u, v, w, x);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(const Quaternion &quat1) {
    T u, v, w, x;

    //a1b2 + b1a2 + c1d2 - d1c2
    u = (this->_x * quat1._u) + (this->_u * quat1._x) + (this->_v * quat1._w) - (this->_w * quat1._v);

    //a1c2 - b1d2 + c1a2 + d1b2
    v = (this->_x * quat1._v) - (this->_u * quat1._w) + (this->_v * quat1._x) + (this->_w * quat1._u);

    //a1d2 + b1c2 - c1b2 + d1a2
    w = (this->_x * quat1._w) + (this->_u * quat1._v) - (this->_v * quat1._w) + (this->_w * quat1._x);

    x = (this->_x * quat1._x) - (this->_u * quat1._u) - (this->_v * quat1._v) - (this->_w * quat1._w);
    return Quaternion<T>(u, v, w, x);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(const T scalefactor) {
    T qu = this->_u * scalefactor;
    T qv = this->_v * scalefactor;
    T qw = this->_w * scalefactor;
    T qx = this->_x * scalefactor;

    return Quaternion<T>(qu, qv, qw, qx);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator+(const Quaternion &quat1) {
    T qu = this->_u + quat1._u;
    T qv = this->_v + quat1._v;
    T qw = this->_w + quat1._w;
    T qx = this->_x + quat1._x;

    return Quaternion<T>(qu, qv, qw, qx);
}

template<typename T>
Quaternion<T> Quaternion<T>::operator-(const Quaternion &quat1) {
    T qu = this->_u - quat1._u;
    T qv = this->_v - quat1._v;
    T qw = this->_w - quat1._w;
    T qx = this->_x - quat1._x;

    return Quaternion<T>(qu, qv, qw, qx);
}

template<typename T>
Quaternion<T> &Quaternion<T>::operator=(const Quaternion &quat1) {
    this->_u = quat1._u;
    this->_v = quat1._v;
    this->_w = quat1._w;
    this->_x = quat1._x;
    return *this;
}


template class Quaternion<float>;