/**
 * @file Matrix2x2.h
 * Contains template class Matrix2x2 of type V
 *
 * @author <a href="mailto:Kai_Engel@gmx.de">Kai Engel</a>
 * @author <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#pragma once

#include "Vector2.h"
#include <limits>

/** This class represents a 2x2-matrix */
template<class V = float>
class Matrix2x2 {
public:
    Vector2<V> c[2];

    Matrix2x2<V>() {
        c[0] = Vector2<V>(1, 0);
        c[1] = Vector2<V>(0, 1);
    }

    Matrix2x2<V>(
            const V &a11, const V &a12,
            const V &a21, const V &a22) {
        c[0].x = a11;
        c[1].x = a12;
        c[0].y = a21;
        c[1].y = a22;
    }

    Matrix2x2<V>(const Vector2<V> &c0, const Vector2<V> &c1) {
        c[0] = c0;
        c[1] = c1;
    }

    Matrix2x2<V> &operator=(const Matrix2x2<V> &other) {
        c[0] = other.c[0];
        c[1] = other.c[1];
        return *this;
    }

    Matrix2x2<V>(const Matrix2x2<V> &other) {
        *this = other;
    }

    Vector2<V> &operator[](int i) {
        return c[i];
    }

    const Vector2<V> &operator[](int i) const {
        return c[i];
    }

    Vector2<V> operator*(const Vector2<V> &vector) const {
        return (c[0] * vector.x + c[1] * vector.y);
    }

    Matrix2x2<V> operator*(const Matrix2x2<V> &other) const {
        Matrix2x2<V> returnMatrix;
        returnMatrix.c[0].x = c[0].x * other.c[0].x + c[1].x * other.c[0].y;
        returnMatrix.c[0].y = c[0].y * other.c[0].x + c[1].y * other.c[0].y;
        returnMatrix.c[1].x = c[0].x * other.c[1].x + c[1].x * other.c[1].y;
        returnMatrix.c[1].y = c[0].y * other.c[1].x + c[1].y * other.c[1].y;
        return returnMatrix;
    }

    Matrix2x2<V> operator*=(const Matrix2x2<V> &other) {
        return *this = *this * other;
    }

    Matrix2x2<V> &operator*=(const V &factor) {
        c[0] *= factor;
        c[1] *= factor;
        return *this;
    }

    Matrix2x2<V> &operator/=(const V &factor) {
        c[0] /= factor;
        c[1] /= factor;
        return *this;
    }

    Matrix2x2<V> operator*(const V &factor) const {
        return Matrix2x2<V>(*this) *= factor;
    }

    Matrix2x2<V> operator/(const V &factor) const {
        return Matrix2x2<V>(*this) /= factor;
    }

    Matrix2x2<V> operator+(const Matrix2x2<V> &other) const {
        return Matrix2x2<V>(
                Vector2<V>(c[0].x + other.c[0].x, c[0].y + other.c[0].y),
                Vector2<V>(c[1].x + other.c[1].x, c[1].y + other.c[1].y));
    }

    Matrix2x2<V> operator-(const Matrix2x2<V> &other) const {
        return Matrix2x2<V>(
                Vector2<V>(c[0].x - other.c[0].x, c[0].y - other.c[0].y),
                Vector2<V>(c[1].x - other.c[1].x, c[1].y - other.c[1].y));
    }

    Matrix2x2<V> &operator+=(const Matrix2x2<V> &other) {
        c[0] += other.c[0];
        c[1] += other.c[1];
        return *this;
    }

    Matrix2x2<V> &operator-=(const Matrix2x2<V> &other) {
        c[0] -= other.c[0];
        c[1] -= other.c[1];
        return *this;
    }

    Matrix2x2<V> invert() const {
        V factor(det());
        if (std::abs(factor) < std::numeric_limits<V>::min())
            factor = std::numeric_limits<V>::min();
        else
            factor = 1.f / factor;
        return Matrix2x2<V>(Vector2<V>(factor * c[1].y, -factor * c[0].y),
                            Vector2<V>(-factor * c[1].x, factor * c[0].x));
    }

    bool operator==(const Matrix2x2<V> &other) const {
        return (c[0] == other.c[0] && c[1] == other.c[1]);
    }

    bool operator!=(const Matrix2x2<V> &other) const {
        return !(*this == other);
    }

    Matrix2x2<V> transpose() const {
        return Matrix2x2<V>(Vector2<V>(c[0].x, c[1].x),
                            Vector2<V>(c[0].y, c[1].y));
    }

    V det() const {
        return c[0].x * c[1].y - c[1].x * c[0].y;
    }

    V trace() const {
        return c[0].x + c[1].y;
    }
};
