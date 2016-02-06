/**
 * @file Pose2D.h
 * Contains class Pose2D
 *
 * @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
 * @author Max Risler
 */

#pragma once

#include "Vector2.h"
#include "BHMath.h"

template<class T>
class Range;

class Pose2D {
public:

    float rotation;

    Vector2<> translation;

    Pose2D() : rotation(0), translation(0, 0) { }

    Pose2D(const float rotation, const Vector2<> &translation) : rotation(
            rotation), translation(translation) { }

    Pose2D(const float rot, const float x, const float y) : rotation(rot),
                                                            translation(x,
                                                                        y) { }

    Pose2D(const float rotation) : rotation(rotation), translation(0, 0) { }

    Pose2D(const Vector2<> &translation) : rotation(0),
                                           translation(translation) { }

    Pose2D(const Vector2<int> &translation) : rotation(0),
                                              translation((float) translation.x,
                                                          (float) translation.y) { }

    Pose2D(const float x, const float y) : rotation(0), translation(x, y) { }

    Pose2D &operator=(const Pose2D &other) {
        rotation = other.rotation;
        translation = other.translation;
        return *this;
    }

    Pose2D(const Pose2D &other) { *this = other; }

    Vector2<> operator*(const Vector2<> &point) const {
        float s = std::sin(rotation);
        float c = std::cos(rotation);
        return (Vector2<>(point.x * c - point.y * s,
                          point.x * s + point.y * c) + translation);
    }

    bool operator==(const Pose2D &other) const {
        return ((translation == other.translation) &&
                (rotation == other.rotation));
    }

    bool operator!=(const Pose2D &other) const { return !(*this == other); }

    Pose2D &operator+=(const Pose2D &other) {
        translation = *this * other.translation;
        rotation += other.rotation;
        rotation = normalize(rotation);
        return *this;
    }

    Pose2D operator+(const Pose2D &other) const {
        return Pose2D(*this) += other;
    }

    Pose2D &operator-=(const Pose2D &other) {
        translation -= other.translation;
        Pose2D p(-other.rotation);
        return *this = p + *this;
    }

    Pose2D operator-(const Pose2D &other) const {
        return Pose2D(*this) -= other;
    }

    Pose2D &conc(const Pose2D &other) { return *this += other; }

    Pose2D &translate(const Vector2<> &trans) {
        translation = *this * trans;
        return *this;
    }

    Pose2D &translate(const float x, const float y) {
        translation = *this * Vector2<>(x, y);
        return *this;
    }

    Pose2D &rotate(const float angle) {
        rotation += angle;
        return *this;
    }

    Pose2D invert() const {
        const float &invRotation = -rotation;
        return Pose2D(invRotation,
                      (Vector2<>() - translation).rotate(invRotation));
    }
};
