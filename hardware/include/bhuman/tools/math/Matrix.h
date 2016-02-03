#pragma once

#include "Vector.h"
#include <assert.h>

template<int m = 2, int n = 2, class V = float>
class Matrix {
public:
    Vector<m, V> c[n];

    Matrix<m, n, V>() = default;

    Matrix<m, n, V>(V v) {
        const int mnm = n < m ? n : m;
        for (int i = 0; i < mnm; ++i)
            c[i][i] = v;
    }

    Matrix<m, n, V>(const Vector<m, V> &c0) {
        assert(n == 1);
        c[0] = c0;
    }

    Matrix<m, n, V>(const Vector<m, V> &c0, const Vector<m, V> &c1) {
        assert(n == 2);
        c[0] = c0;
        c[1] = c1;
    }

    Matrix<m, n, V>(const Vector<m, V> &c0, const Vector<m, V> &c1,
                    const Vector<m, V> &c2) {
        assert(n == 3);
        c[0] = c0;
        c[1] = c1;
        c[2] = c2;
    }

    Matrix<m, n, V>(const Vector<m, V> &c0, const Vector<m, V> &c1,
                    const Vector<m, V> &c2, const Vector<m, V> &c3) {
        assert(n == 4);
        c[0] = c0;
        c[1] = c1;
        c[2] = c2;
        c[3] = c3;
    }

    Matrix<m, n, V> &operator=(const Matrix<m, n, V> &other) {
        for (int i = 0; i < n; ++i)
            c[i] = other.c[i];
        return *this;
    }

    Vector<m, V> &operator[](int i) {
        return c[i];
    }

    const Vector<m, V> &operator[](int i) const {
        return c[i];
    }

    Vector<m, V> operator*(const Vector<n, V> &vector) const {
        Vector<m, V> result = c[0] * vector[0];
        for (int i = 1; i < n; ++i)
            result += c[i] * vector[i];
        return result;
    }

    template<int o>
    Matrix<m, o, V> operator*(const Matrix<n, o, V> &other) const {
        Matrix<m, o, V> result;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j)
                for (int k = 0; k < o; ++k)
                    result.c[k][j] += c[i][j] * other.c[k][i];
        return result;
    }

    Matrix<n, n, V> operator*=(const Matrix<n, n, V> &other) {
        return *this = *this * other;
    }

    Matrix<m, n, V> &operator*=(const V &factor) {
        for (int i = 0; i < n; ++i)
            c[i] *= factor;
        return *this;
    }

    Matrix<m, n, V> operator*(const V &factor) const {
        return Matrix<m, n, V>(*this) *= factor;
    }

    Matrix<m, n, V> &operator/=(const V &factor) {
        for (int i = 0; i < n; ++i)
            c[i] /= factor;
        return *this;
    }

    Matrix<m, n, V> operator/(const V &factor) const {
        return Matrix<m, n, V>(*this) /= factor;
    }

    Matrix<m, n, V> &operator+=(const Matrix<m, n, V> &other) {
        for (int i = 0; i < n; ++i)
            c[i] += other.c[i];
        return *this;
    }

    Matrix<m, n, V> operator+(const Matrix<m, n, V> &other) const {
        return Matrix<m, n, V>(*this) += other;
    }

    Matrix<m, n, V> &operator-=(const Matrix<m, n, V> &other) {
        for (int i = 0; i < n; ++i)
            c[i] -= other.c[i];
        return *this;
    }

    Matrix<m, n, V> operator-(const Matrix<m, n, V> &other) const {
        return Matrix<m, n, V>(*this) -= other;
    }

    bool operator==(const Matrix<m, n, V> &other) const {
        for (int i = 0; i < n; ++i)
            if (c[i] != other.c[i])
                return false;
        return true;
    }

    bool operator!=(const Matrix<m, n, V> &other) const {
        for (int i = 0; i < n; ++i)
            if (c[i] != other.c[i])
                return true;
        return false;
    }

    Matrix<n, m, V> transpose() const {
        Matrix<n, m, V> result;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < m; ++j)
                result.c[j][i] = c[i][j];
        return result;
    }

    V det() const;

    Matrix<m, n, V> adjoint() const;

    Matrix<m, n, V> invert() const;

    bool solve(const Vector<n, V> &b, Vector<n, V> &x) const;
};

typedef Matrix<1, 1, float> Matrix1x1f;
typedef Matrix<2, 2, float> Matrix2x2f;
typedef Matrix<2, 1, float> Matrix2x1f;
typedef Matrix<1, 2, float> Matrix1x2f;
typedef Matrix<3, 3, float> Matrix3x3f;
typedef Matrix<3, 2, float> Matrix3x2f;
typedef Matrix<3, 1, float> Matrix3x1f;
typedef Matrix<2, 3, float> Matrix2x3f;
typedef Matrix<1, 3, float> Matrix1x3f;
typedef Matrix<4, 4, float> Matrix4x4f;
typedef Matrix<4, 3, float> Matrix4x3f;
typedef Matrix<4, 2, float> Matrix4x2f;
typedef Matrix<4, 1, float> Matrix4x1f;
typedef Matrix<3, 4, float> Matrix3x4f;
typedef Matrix<2, 4, float> Matrix2x4f;
typedef Matrix<1, 4, float> Matrix1x4f;
