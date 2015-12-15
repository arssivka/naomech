/**
 * @file Vector2.h
 * Contains template class Vector2 of type V
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#pragma once

#include <cmath>

/** This class represents a 2-vector */
template<class V = float>
class Vector2 {
public:
    V x, y;

    Vector2<V>() : x(V()), y(V()) { }

    Vector2<V>(V x, V y) : x(x), y(y) { }

    Vector2<V>(const Vector2<V> &other) : x(other.x), y(other.y) { }

    template<typename O>
    inline explicit Vector2<V>(const Vector2<O> &other) : x((V) other.x),
                                                          y((V) other.y) { }

    Vector2<V> &operator=(const Vector2<V> &other) {
        x = other.x;
        y = other.y;
        return *this;
    }

    Vector2<V> &operator+=(const Vector2<V> &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2<V> &operator-=(const Vector2<V> &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2<V> &operator*=(const V &factor) {
        x *= factor;
        y *= factor;
        return *this;
    }

    Vector2<V> &operator/=(const V &factor) {
        if (factor == V())
            return *this;
        x /= factor;
        y /= factor;
        return *this;
    }

    Vector2<V> operator+(const Vector2<V> &other) const {
        return Vector2<V>(*this) += other;
    }

    Vector2<V> operator-(const Vector2<V> &other) const {
        return Vector2<V>(*this) -= other;
    }

    Vector2<V> operator-() const {
        return Vector2<V>(-x, -y);
    }

    V operator*(const Vector2<V> &other) const {
        return x * other.x + y * other.y;
    }

    Vector2<V> operator*(const V &factor) const {
        return Vector2<V>(*this) *= factor;
    }

    Vector2<V> operator/(const V &factor) const {
        return Vector2<V>(*this) /= factor;
    }

    bool operator==(const Vector2<V> &other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vector2<V> &other) const {
        return x != other.x || y != other.y;
    }

    V abs() const {
        return (V) std::sqrt(((float) x) * x + ((float) y) * y);
    }

    float absFloat() const {
        return std::sqrt(((float) x) * x + ((float) y) * y);
    }

    V squareAbs() const {
        return x * x + y * y;
    }

    V sqr() const {
        return x * x + y * y;
    }

    Vector2<V> &normalize(V len) {
        const V length = abs();
        if (length == V())
            return *this;
        *this *= len;
        return *this /= length;
    }

    Vector2<V> &normalize() {
        return *this /= abs();
    }

    Vector2<V> &rotateLeft() {
        V buffer = -y;
        y = x;
        x = buffer;
        return *this;
    }

    Vector2<V> &rotateRight() {
        V buffer = -x;
        x = y;
        y = buffer;
        return *this;
    }

    Vector2<V> &mirror() {
        x = -x;
        y = -y;
        return *this;
    }

    Vector2<V> &rotate(float alpha) {
        float buffer = (float) x;
        float a = std::cos(alpha);
        float b = std::sin(alpha);
        x = (V) (a * (float) x - b * (float) y);
        y = (V) (b * buffer + a * (float) y);
        return *this;
    }

    V &operator[](int i) {
        return (&x)[i];
    }

    const V &operator[](int i) const {
        return (&x)[i];
    }

    float angle() const {
        return std::atan2((float) y, (float) x);
    }
};