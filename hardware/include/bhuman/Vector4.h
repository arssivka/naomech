#pragma once

#include <cmath>

template<class V = float>
class Vector4 {
public:
    V x, y, z, w;

    Vector4<V>() : x(V()), y(V()), z(V()), w(V()) { }

    Vector4<V>(V x, V y, V z, V w) : x(x), y(y), z(z), w(w) { }

    Vector4<V>(const Vector4<V> &other) : x(other.x), y(other.y), z(other.z),
                                          w(other.w) { }

    template<typename O>
    inline explicit Vector4<V>(const Vector4<O> &other) : x((V) other.x),
                                                          y((V) other.y),
                                                          z((V) other.z),
                                                          w((V) other.w) { }

    Vector4<V> &operator=(const Vector4<V> &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
        return *this;
    }

    Vector4<V> &operator+=(const Vector4<V> &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        w += other.w;
        return *this;
    }

    Vector4<V> &operator-=(const Vector4<V> &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        w -= other.w;
        return *this;
    }

    Vector4<V> &operator*=(const V &factor) {
        x *= factor;
        y *= factor;
        z *= factor;
        w *= factor;
        return *this;
    }

    Vector4<V> &operator/=(const V &factor) {
        if (factor == V())
            return *this;
        x /= factor;
        y /= factor;
        z /= factor;
        w /= factor;
        return *this;
    }

    Vector4<V> operator+(const Vector4<V> &other) const {
        return Vector4<V>(*this) += other;
    }

    Vector4<V> operator-(const Vector4<V> &other) const {
        return Vector4<V>(*this) -= other;
    }

    Vector4<V> operator-() const {
        return Vector4<V>(-x, -y, -z, -w);
    }

    V operator*(const Vector4<V> &other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    Vector4<V> operator*(const V &factor) const {
        return Vector4<V>(*this) *= factor;
    }

    Vector4<V> operator/(const V &factor) const {
        return Vector4<V>(*this) /= factor;
    }

    bool operator==(const Vector4<V> &other) const {
        return x == other.x && y == other.y && z == other.z && w == other.w;
    }

    bool operator!=(const Vector4<V> &other) const {
        return x != other.x || y != other.y || z != other.z || w != other.w;
    }

    V &operator[](int i) {
        return (&x)[i];
    }

    const V &operator[](int i) const {
        return (&x)[i];
    }

    V abs() const {
        return (V) sqrt(float((x * x) + (y * y) + (z * z) + (w * w)));
    }

    V squareAbs() const {
        return x * x + y * y + z * z + w * w;
    }

    Vector4<V> &normalize(V len) {
        const V length = abs();
        if (length == V())
            return *this;
        *this *= len;
        return *this /= length;
    }

    Vector4<V> &normalize() {
        return *this /= abs();
    }
};
