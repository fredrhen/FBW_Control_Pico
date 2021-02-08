//
// Created by Freds on 2021-02-06.
//

#ifndef FBW_CONTROL_PICO_VECTOR_H
#define FBW_CONTROL_PICO_VECTOR_H

#include "pico/stdlib.h"
#include "math.h"


template<typename T, int len>
class Vector {
public:
    Vector() {
        this->_array = new T[len];
    }

    template<typename ... Args>
    Vector(T arg1, Args ... arg2) {
        this->_array = new T[len];
        fold_fill(0, arg1, arg2...);

    }

    Vector(const Vector<T, len> &v2 ){
        for(int i=0; i<len; ++i) {
            this[i] = v2[i];
        }
    }

    ~Vector(){
        delete[] _array;
        _array = nullptr;
    }
    constexpr  int get_len() const {
        return len;
    }

    T magnitude() const {
        T mag = 0;
        for (int i = 0; i < this->get_len(); ++i) {
            mag += _array[i] * _array[i];
        }

        return sqrt(mag);
    }

    void normalize() const {
        T mag = this->magnitude();
        T temp = 1/mag;
        for (int i = 0; i < this->get_len(); ++i) {
            this->_array[i] *= temp;
        }
    }

    void normalize(T &mag) const {
        mag = this->magnitude();
        T temp = 1/mag;
        for (int i = 0; i < this->get_len(); ++i) {
            this->_array[i] *= temp;
        }
    }

    static T dot(const Vector &v1, const Vector &v2) {
        T dot = 0;
        for (int i = 0; i < v1.get_len(); ++i) {
            dot += (v1[i] * v2[i]);
        }
        return dot;
    }

    // Cross product which only works for len=3 vectors
    template<typename t>
    static Vector<t, 3> cross(const Vector<t, 3>&v1, const Vector<t, 3> &v2) {
        Vector<T, 3> cross;

        cross[0] = v1[1] * v2[2] - v1[2] * v2[1];
        cross[1] = v1[2] * v2[0] - v1[0] * v2[2];
        cross[2] = v1[0] * v2[1] - v1[1] * v2[0];

        return cross;
    }

    T &operator[](int b) {
        return this->_array[b];
    }

    T operator[](int b) const {
        return this->_array[b];
    }

    Vector &operator=(const Vector &v2)
    {
        for(int i=0; i<get_len(); ++i)
        {
            _array[i] = v2[i];
        }
        return *this;
    }

    Vector operator+(const Vector &v2) const{
        Vector<T, len> v3;
        for(int i=0; i<get_len(); ++i)
        {
            v3[i] = this->_array[i] + v2[i];
        }

        return v3;
    }

    Vector operator-(const Vector &v2) const{
        Vector<T, len> v3;
        for(int i=0; i<get_len(); ++i)
        {
            v3[i] = this->_array[i] - v2[i];
        }

        return v3;
    }

    Vector operator-() const {
        Vector<T, len> v3;
        for(int i=0; i<get_len(); ++i)
        {
            v3[i] = -this->_array[i];
        }
        return v3;
    }
    template<typename ts>
    Vector operator*(ts scalar) const {
        Vector<T, len> v3;
        for(int i=0; i<get_len(); ++i){
            v3[i] = this->_array[i] * (T)scalar;
        }

        return v3;
    }

    template<typename ts>
    Vector& operator*=(ts scalar) {
        for(int i=0; i<get_len(); ++i){
            this->_array[i] *= (T)scalar;
        }

        return *this;
    }



private:
    T* _array;

    void fold_fill(int index, T arg1){
        this->_array[index] = arg1;
    }

    template<typename ... Args>
    void fold_fill(int index, T arg1, Args ... arg2){
        this->_array[index++] = arg1;
        fold_fill(index, arg2...);
    }


};



#endif //FBW_CONTROL_PICO_VECTOR_H