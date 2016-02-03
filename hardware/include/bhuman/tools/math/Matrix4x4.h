/**
* @file Matrix4x4.h
* Contains template class Matrix4x4 of type V
* @author <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
* @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
* @author Colin Graf
* @editor Florian Maaß
*/

#pragma once

#include "Vector4.h"

template<class V = float>
class Matrix4x4;

template<class V>
class Matrix4x4 {
public:
    Vector4<V> c0;
    Vector4<V> c1;
    Vector4<V> c2;
    Vector4<V> c3;

    Matrix4x4<V>() : c0(1, 0, 0, 0), c1(0, 1, 0, 0), c2(0, 0, 1, 0),
                     c3(0, 0, 0, 1) { }

    Matrix4x4<V>(const Vector4<V> &c0, const Vector4<V> &c1,
                 const Vector4<V> &c2, const Vector4<V> &c3) : c0(c0), c1(c1),
                                                               c2(c2),
                                                               c3(c3) { }

    Matrix4x4<V>(const Matrix4x4<V> &other) : c0(other.c0), c1(other.c1),
                                              c2(other.c2), c3(other.c3) { }

    Matrix4x4<V>(
            const V &a11, const V &a12, const V &a13, const V &a14,
            const V &a21, const V &a22, const V &a23, const V &a24,
            const V &a31, const V &a32, const V &a33, const V &a34,
            const V &a41, const V &a42, const V &a43, const V &a44)
            : c0(a11, a21, a31, a41), c1(a12, a22, a32, a42),
              c2(a13, a23, a33, a43), c3(a14, a24, a34, a44) { }

    Matrix4x4<V> &operator=(const Matrix4x4<V> &other) {
        c0 = other.c0;
        c1 = other.c1;
        c2 = other.c2;
        c3 = other.c3;
        return *this;
    }

    Matrix4x4<V> operator+(const Matrix4x4<V> &other) const {
        return Matrix4x4<V>(
                c0 + other.c0,
                c1 + other.c1,
                c2 + other.c2,
                c3 + other.c3
        );
    }

    Matrix4x4<V> &operator+=(const Matrix4x4<V> &other) {
        c0 += other.c0;
        c1 += other.c1;
        c2 += other.c2;
        c3 += other.c3;
        return *this;
    }

    Matrix4x4<V> operator-(const Matrix4x4<V> &other) const {
        return Matrix4x4<V>(
                c0 - other.c0,
                c1 - other.c1,
                c2 - other.c2,
                c3 - other.c3
        );
    }

    Matrix4x4<V> &operator-=(const Matrix4x4<V> &other) {
        c0 -= other.c0;
        c1 -= other.c1;
        c2 -= other.c2;
        c3 -= other.c3;
        return *this;
    }

    Vector4<V> operator*(const Vector4<V> &vector) const {
        return Vector4<V>(
                c0.x * vector.x + c1.x * vector.y + c2.x * vector.z,
                c3.x * vector.w,
                c0.y * vector.x + c1.y * vector.y + c2.y * vector.z,
                c3.y * vector.w,
                c0.z * vector.x + c1.z * vector.y + c2.z * vector.z,
                c3.z * vector.w,
                c0.w * vector.x + c1.w * vector.y + c2.w * vector.z,
                c3.w * vector.w);
    }

    Matrix4x4<V> operator*(const Matrix4x4<V> &other) const {
        Matrix4x4<V> result;
        result.c0.x =
                c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z +
                c3.x * other.c0.w;
        result.c0.y =
                c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z +
                c3.y * other.c0.w;
        result.c0.z =
                c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z +
                c3.z * other.c0.w;
        result.c0.w =
                c0.w * other.c0.x + c1.w * other.c0.y + c2.w * other.c0.z +
                c3.w * other.c0.w;
        result.c1.x =
                c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z +
                c3.x * other.c1.w;
        result.c1.y =
                c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z +
                c3.y * other.c1.w;
        result.c1.z =
                c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z +
                c3.z * other.c1.w;
        result.c1.w =
                c0.w * other.c1.x + c1.w * other.c1.y + c2.w * other.c1.z +
                c3.w * other.c1.w;
        result.c2.x =
                c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z +
                c3.x * other.c2.w;
        result.c2.y =
                c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z +
                c3.y * other.c2.w;
        result.c2.z =
                c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z +
                c3.z * other.c2.w;
        result.c2.w =
                c0.w * other.c2.x + c1.w * other.c2.y + c2.w * other.c2.z +
                c3.w * other.c2.w;
        result.c3.x =
                c0.x * other.c3.x + c1.x * other.c3.y + c2.x * other.c3.z +
                c3.x * other.c3.w;
        result.c3.y =
                c0.y * other.c3.x + c1.y * other.c3.y + c2.y * other.c3.z +
                c3.y * other.c3.w;
        result.c3.z =
                c0.z * other.c3.x + c1.z * other.c3.y + c2.z * other.c3.z +
                c3.z * other.c3.w;
        result.c3.w =
                c0.w * other.c3.x + c1.w * other.c3.y + c2.w * other.c3.z +
                c3.w * other.c3.w;
        return result;
    }

    Matrix4x4<V> &operator*=(const Matrix4x4<V> &other) {
        Matrix4x4<V> result;
        result.c0.x =
                c0.x * other.c0.x + c1.x * other.c0.y + c2.x * other.c0.z +
                c3.x * other.c0.w;
        result.c0.y =
                c0.y * other.c0.x + c1.y * other.c0.y + c2.y * other.c0.z +
                c3.y * other.c0.w;
        result.c0.z =
                c0.z * other.c0.x + c1.z * other.c0.y + c2.z * other.c0.z +
                c3.z * other.c0.w;
        result.c0.w =
                c0.w * other.c0.x + c1.w * other.c0.y + c2.w * other.c0.z +
                c3.w * other.c0.w;
        result.c1.x =
                c0.x * other.c1.x + c1.x * other.c1.y + c2.x * other.c1.z +
                c3.x * other.c1.w;
        result.c1.y =
                c0.y * other.c1.x + c1.y * other.c1.y + c2.y * other.c1.z +
                c3.y * other.c1.w;
        result.c1.z =
                c0.z * other.c1.x + c1.z * other.c1.y + c2.z * other.c1.z +
                c3.z * other.c1.w;
        result.c1.w =
                c0.w * other.c1.x + c1.w * other.c1.y + c2.w * other.c1.z +
                c3.w * other.c1.w;
        result.c2.x =
                c0.x * other.c2.x + c1.x * other.c2.y + c2.x * other.c2.z +
                c3.x * other.c2.w;
        result.c2.y =
                c0.y * other.c2.x + c1.y * other.c2.y + c2.y * other.c2.z +
                c3.y * other.c2.w;
        result.c2.z =
                c0.z * other.c2.x + c1.z * other.c2.y + c2.z * other.c2.z +
                c3.z * other.c2.w;
        result.c2.w =
                c0.w * other.c2.x + c1.w * other.c2.y + c2.w * other.c2.z +
                c3.w * other.c2.w;
        result.c3.x =
                c0.x * other.c3.x + c1.x * other.c3.y + c2.x * other.c3.z +
                c3.x * other.c3.w;
        result.c3.y =
                c0.y * other.c3.x + c1.y * other.c3.y + c2.y * other.c3.z +
                c3.y * other.c3.w;
        result.c3.z =
                c0.z * other.c3.x + c1.z * other.c3.y + c2.z * other.c3.z +
                c3.z * other.c3.w;
        result.c3.w =
                c0.w * other.c3.x + c1.w * other.c3.y + c2.w * other.c3.z +
                c3.w * other.c3.w;
        *this = result;
        return *this;
    }

    Matrix4x4<V> &operator*=(const V &factor) {
        c0 *= factor;
        c1 *= factor;
        c2 *= factor;
        c3 *= factor;
        return *this;
    }

    Matrix4x4<V> &operator/=(const V &factor) {
        c0 /= factor;
        c1 /= factor;
        c2 /= factor;
        c3 /= factor;
        return *this;
    }

    Matrix4x4<V> operator*(const V &factor) const {
        return Matrix4x4<V>(c0 * factor, c1 * factor, c2 * factor, c3 * factor);
    }

    Matrix4x4<V> operator/(const V &factor) const {
        return Matrix4x4<V>(*this) /= factor;
    }

    bool operator==(const Matrix4x4<V> &other) const {
        return c0 == other.c0 && c1 == other.c1 && c2 == other.c2 &&
               c3 == other.c3;
    }

    bool operator!=(const Matrix4x4<V> &other) const {
        return c0 != other.c0 || c1 != other.c1 || c2 != other.c2 ||
               c3 != other.c3;
    }

    Vector4<V> &operator[](int i) {
        return (&c0)[i];
    }

    const Vector4<V> &operator[](int i) const {
        return (&c0)[i];
    }

    Matrix4x4<V> transpose() const {
        return Matrix4x4<V>(
                Vector4<V>(c0.x, c1.x, c2.x, c3.x),
                Vector4<V>(c0.y, c1.y, c2.y, c3.y),
                Vector4<V>(c0.z, c1.z, c2.z, c3.z),
                Vector4<V>(c0.w, c1.w, c2.w, c3.w)
        );
    }
};
