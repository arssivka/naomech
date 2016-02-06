/**
* @file Vector.h
* Contains template class Vector of type V and size n
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
* @author Colin Graf
*/

#pragma once

#include <assert.h>
#include <math.h>

namespace vec {

    template<int n, class V>
    struct VectorData {
        V v[n];
    };

    template<class V>
    struct VectorData<2, V> {
        V x;
        V y;
    };

    template<class V>
    struct VectorData<3, V> {
        V x;
        V y;
        V z;
    };

/** This class represents a n-dimensional vector */
    template<int n = 2, class V = float>
    class Vector : public VectorData<n, V> {
    public:
        Vector<n, V>() {
            for (int i = 0; i < n; ++i)
                (*this)[i] = V();
        }

        Vector<n, V>(V x) {
            assert(n == 1);
            (*this)[0] = x;
        }

        Vector<n, V>(V x, V y) {
            assert(n == 2);
            (*this)[0] = x;
            (*this)[1] = y;
        }

        Vector<n, V>(V x, V y, V z) {
            assert(n == 3);
            (*this)[0] = x;
            (*this)[1] = y;
            (*this)[2] = z;
        }

        Vector<n, V>(V x, V y, V z, V w) {
            assert(n == 4);
            (*this)[0] = x;
            (*this)[1] = y;
            (*this)[2] = z;
            (*this)[3] = w;
        }

        Vector<n, V>(const Vector<n, V> &other) {
            for (int i = 0; i < n; ++i)
                (*this)[i] = other[i];
        }

        Vector<n, V> &operator=(const Vector<n, V> &other) {
            for (int i = 0; i < n; ++i)
                (*this)[i] = other[i];
            return *this;
        }

        Vector<n, V> &operator+=(const Vector<n, V> &other) {
            for (int i = 0; i < n; ++i)
                (*this)[i] += other[i];
            return *this;
        }

        Vector<n, V> &operator-=(const Vector<n, V> &other) {
            for (int i = 0; i < n; ++i)
                (*this)[i] -= other[i];
            return *this;
        }

        Vector<n, V> &operator*=(const V &factor) {
            for (int i = 0; i < n; ++i)
                (*this)[i] *= factor;
            return *this;
        }

        Vector<n, V> &operator/=(const V &factor) {
            for (int i = 0; i < n; ++i)
                (*this)[i] /= factor;
            return *this;
        }

        Vector<n, V> operator+(const Vector<n, V> &other) const {
            return Vector<n, V>(*this) += other;
        }

        Vector<n, V> operator-(const Vector<n, V> &other) const {
            return Vector<n, V>(*this) -= other;
        }

        V operator*(const Vector<n, V> &other) const {
            V result = (*this)[0] * other[0];
            for (int i = 1; i < n; ++i)
                result += (*this)[i] * other[i];
            return result;
        }

        Vector<n, V> operator*(const V &factor) const {
            return Vector<n, V>(*this) *= factor;
        }

        Vector<n, V> operator/(const V &factor) const {
            return Vector<n, V>(*this) /= factor;
        }

        Vector<n, V> operator-() const {
            return Vector<n, V>() -= *this;
        }

        bool operator==(const Vector<n, V> &other) const {
            for (int i = 0; i < n; ++i)
                if ((*this)[i] != other[i])
                    return false;
            return true;
        }

        bool operator!=(const Vector<n, V> &other) const {
            for (int i = 0; i < n; ++i)
                if ((*this)[i] != other[i])
                    return true;
            return false;
        }

        V &operator[](int i) {
            return ((V *) this)[i];
        }

        const V &operator[](int i) const {
            return ((const V *) this)[i];
        }

        V sqr() const {
            V result = (*this)[0] * (*this)[0];
            for (int i = 1; i < n; ++i)
                result += (*this)[i] * (*this)[i];
            return result;
        }

        V abs() const {
            return sqrt(sqr());
        }

        Vector<n, V> &normalize() {
            const V length = abs();
            if (length == V())
                return *this;
            else
                return *this /= length;
        }
    };
}

using vec::Vector;

typedef Vector<1, float> Vector1f;
typedef Vector<2, float> Vector2f;
typedef Vector<3, float> Vector3f;
typedef Vector<4, float> Vector4f;

typedef Vector<2, int> Vector2i;
typedef Vector<3, int> Vector3i;
typedef Vector<4, int> Vector4i;