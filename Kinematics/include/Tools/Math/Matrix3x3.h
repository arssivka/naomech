/**
* @file Matrix3x3.h
* Contains template class Matrix3x3 of type V
* @author <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
* @author Colin Graf
*/

#pragma once

#include "Vector3.h"

template<class V = float>
class Matrix3x3;

/**
 * This class represents a 3x3-matrix
 *
 */
template<class V>
class Matrix3x3 {
public:
    Vector3<V> c0;
    Vector3<V> c1;
    Vector3<V> c2;

    Matrix3x3<V>() : c0(1, 0, 0), c1(0, 1, 0), c2(0, 0, 1) { }

    Matrix3x3<V>(const Vector3<V> &c0, const Vector3<V> &c1,
                 const Vector3<V> &c2) : c0(c0), c1(c1), c2(c2) { }

    Matrix3x3<V>(const Matrix3x3<V> &other) : c0(other.c0), c1(other.c1),
                                              c2(other.c2) { }

    Matrix3x3<V>(
            const V &a11, const V &a12, const V &a13,
            const V &a21, const V &a22, const V &a23,
            const V &a31, const V &a32, const V &a33) : c0(a11, a21, a31),
                                                        c1(a12, a22, a32),
                                                        c2(a13, a23, a33) { }

    Matrix3x3<V> &operator=(const Matrix3x3<V> &other) {
        c0 = other.c0;
        c1 = other.c1;
        c2 = other.c2;
        return *this;
    }

    Matrix3x3<V> operator+(const Matrix3x3<V> &other) const {
        return Matrix3x3<V>(
                c0 + other.c0,
                c1 + other.c1,
                c2 + other.c2
        );
    }

    Matrix3x3<V> &operator+=(const Matrix3x3<V> &other) {
        c0 += other.c0;
        c1 += other.c1;
        c2 += other.c2;
        return *this;
    }

    Matrix3x3<V> operator-(const Matrix3x3<V> &other) const {
        return Matrix3x3<V>(
                c0 - other.c0,
                c1 - other.c1,
                c2 - other.c2
        );
    }

    Matrix3x3<V> &operator-=(const Matrix3x3<V> &other) {
        c0 -= other.c0;
        c1 -= other.c1;
        c2 -= other.c2;
        return *this;
    }

    Vector3<V> operator*(const Vector3<V> &vector) const {
        /*
        return c0 * vector.x + c1 * vector.y + c2 * vector.z;
        */
        return Vector3<V>(
                c0.x * vector.x + c1.x * vector.y + c2.x * vector.z,
                c0.y * vector.x + c1.y * vector.y + c2.y * vector.z,
                c0.z * vector.x + c1.z * vector.y + c2.z * vector.z);
    }

    Matrix3x3<V> operator*(const Matrix3x3<V> &other) const {
        Matrix3x3<V> result;
        result.c0.x = c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z;
        result.c0.y = c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z;
        result.c0.z = c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z;
        result.c1.x = c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z;
        result.c1.y = c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z;
        result.c1.z = c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z;
        result.c2.x = c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z;
        result.c2.y = c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z;
        result.c2.z = c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z;
        return result;
    }

    Matrix3x3<V> &operator*=(const Matrix3x3<V> &other) {
        Matrix3x3<V> result;
        result.c0.x = c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z;
        result.c0.y = c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z;
        result.c0.z = c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z;
        result.c1.x = c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z;
        result.c1.y = c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z;
        result.c1.z = c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z;
        result.c2.x = c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z;
        result.c2.y = c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z;
        result.c2.z = c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z;
        *this = result;
        return *this;
    }

    Matrix3x3<V> &operator*=(const V &factor) {
        c0 *= factor;
        c1 *= factor;
        c2 *= factor;
        return *this;
    }

    Matrix3x3<V> &operator/=(const V &factor) {
        c0 /= factor;
        c1 /= factor;
        c2 /= factor;
        return *this;
    }

    Matrix3x3<V> operator*(const V &factor) const {
        return Matrix3x3<V>(c0 * factor, c1 * factor, c2 * factor);
    }

    Matrix3x3<V> operator/(const V &factor) const {
        return Matrix3x3<V>(*this) /= factor;
    }

    bool operator==(const Matrix3x3<V> &other) const {
        return c0 == other.c0 && c1 == other.c1 && c2 == other.c2;
    }

    bool operator!=(const Matrix3x3<V> &other) const {
        return c0 != other.c0 || c1 != other.c1 || c2 != other.c2;
    }

    Vector3<V> &operator[](int i) {
        return (&c0)[i];
    }

    const Vector3<V> &operator[](int i) const {
        return (&c0)[i];
    }

    Matrix3x3<V> transpose() const {
        return Matrix3x3<V>(
                Vector3<V>(c0.x, c1.x, c2.x),
                Vector3<V>(c0.y, c1.y, c2.y),
                Vector3<V>(c0.z, c1.z, c2.z)
        );
    }

    V det() const {
        return
                c0.x * (c1.y * c2.z - c1.z * c2.y) +
                c0.y * (c1.z * c2.x - c1.x * c2.z) +
                c0.z * (c1.x * c2.y - c1.y * c2.x);
    }

    static V det2(V a, V b, V c, V d) {
        return a * d - b * c;
    }

    Matrix3x3<V> adjoint() const {
        return Matrix3x3<V>(
                Vector3<V>(
                        det2(c1.y, c2.y, c1.z, c2.z),
                        det2(c2.x, c1.x, c2.z, c1.z),
                        det2(c1.x, c2.x, c1.y, c2.y)
                ),
                Vector3<V>(
                        det2(c2.y, c0.y, c2.z, c0.z),
                        det2(c0.x, c2.x, c0.z, c2.z),
                        det2(c2.x, c0.x, c2.y, c0.y)
                ),
                Vector3<V>(
                        det2(c0.y, c1.y, c0.z, c1.z),
                        det2(c1.x, c0.x, c1.z, c0.z),
                        det2(c0.x, c1.x, c0.y, c1.y)
                )
        );
    }

    Matrix3x3<V> invert() const {
        return adjoint().transpose() / det();
    }
};
