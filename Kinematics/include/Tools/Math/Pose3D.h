/**
* @file Pose3D.h
* Contains class Pose3D
* @author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* @author Max Risler
*/

#pragma once

#include "RotationMatrix.h"

/** representation for 3D Transformation (Location + Orientation)*/
class Pose3D {
public:

    RotationMatrix rotation;

    Vector3<> translation;

    Pose3D() = default;

    Pose3D(const RotationMatrix &rot, const Vector3<> &trans) : rotation(rot),
                                                                translation(
                                                                        trans) { }

    Pose3D(const RotationMatrix &rot) : rotation(rot) { }

    Pose3D(const Vector3<> &trans) : translation(trans) { }

    Pose3D(const float x, const float y, const float z) : translation(x, y,
                                                                      z) { }

    Pose3D(const Pose3D &other) : rotation(other.rotation),
                                  translation(other.translation) { }

    Pose3D &operator=(const Pose3D &other) {
        rotation = other.rotation;
        translation = other.translation;
        return *this;
    }

    Vector3<> operator*(const Vector3<> &point) const {
        return rotation * point + translation;
    }

    bool operator==(const Pose3D &other) const {
        return translation == other.translation && rotation == other.rotation;
    }

    bool operator!=(const Pose3D &other) const {
        return translation != other.translation || rotation != other.rotation;
    }

    Pose3D &conc(const Pose3D &other) {
        translation = *this * other.translation;
        rotation *= other.rotation;
        return *this;
    }

    Pose3D invert() const {
        Pose3D result;
        result.rotation = rotation.invert();
        result.translation = result.rotation * (-translation);
        return result;
    }

    Pose3D &translate(const Vector3<> &trans) {
        translation = *this * trans;
        return *this;
    }

    Pose3D &translate(const float x, const float y, const float z) {
        translation = *this * Vector3<>(x, y, z);
        return *this;
    }

    Pose3D &rotate(const RotationMatrix &rot) {
        rotation *= rot;
        return *this;
    }

    Pose3D &rotateX(const float angle) {
        rotation.rotateX(angle);
        return *this;
    }

    Pose3D &rotateY(const float angle) {
        rotation.rotateY(angle);
        return *this;
    }

    Pose3D &rotateZ(const float angle) {
        rotation.rotateZ(angle);
        return *this;
    }
};
