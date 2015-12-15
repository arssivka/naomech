/**
 * @file Vector3.h
 * Contains template class Vector3 of type V
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#pragma once

#include <cmath>

template<class V = float>
class Vector3 {
public:
    V x, y, z;

    Vector3<V>() : x(V()), y(V()), z(V()) { }

    Vector3<V>(V x, V y, V z) : x(x), y(y), z(z) { }

    Vector3<V>(const Vector3<V> &other) : x(other.x), y(other.y), z(other.z) { }

    template<typename O>
    inline explicit Vector3<V>(const Vector3<O> &other) : x((V) other.x),
                                                          y((V) other.y),
                                                          z((V) other.z) { }

    Vector3<V> &operator=(const Vector3<V> &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    Vector3<V> &operator+=(const Vector3<V> &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3<V> &operator-=(const Vector3<V> &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3<V> &operator*=(const V &factor) {
        x *= factor;
        y *= factor;
        z *= factor;
        return *this;
    }

    Vector3<V> &operator/=(const V &factor) {
        if (factor == V())
            return *this;
        x /= factor;
        y /= factor;
        z /= factor;
        return *this;
    }

    Vector3<V> operator+(const Vector3<V> &other) const {
        return Vector3<V>(*this) += other;
    }

    Vector3<V> operator-(const Vector3<V> &other) const {
        return Vector3<V>(*this) -= other;
    }

    Vector3<V> operator-() const {
        return Vector3<V>(-x, -y, -z);
    }

    V operator*(const Vector3<V> &other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector3<V> operator*(const V &factor) const {
        return Vector3<V>(*this) *= factor;
    }

    Vector3<V> operator/(const V &factor) const {
        return Vector3<V>(*this) /= factor;
    }

    bool operator==(const Vector3<V> &other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const Vector3<V> &other) const {
        return x != other.x || y != other.y || z != other.z;
    }

    V &operator[](int i) {
        return (&x)[i];
    }

    const V &operator[](int i) const {
        return (&x)[i];
    }

    V abs() const {
        return (V) sqrt(float((x * x) + (y * y) + (z * z)));
    }

    V squareAbs() const {
        return x * x + y * y + z * z;
    }

    Vector3<V> operator^(const Vector3<V> &other) const {
        return Vector3<V>(y * other.z - z * other.y, z * other.x - x * other.z,
                          x * other.y - y * other.x);
    }

    Vector3<V> &operator^=(const Vector3<V> &other) {
        return *this = *this ^ other;
    }

    Vector3<V> &normalize(V len) {
        const V length = abs();
        if (length == V())
            return *this;
        *this *= len;
        return *this /= length;
    }

    Vector3<V> &normalize() {
        return *this /= abs();
    }
};