/**
* \file RotationMatrix.h
* Delcaration of class RotationMatrix
* \author <a href="mailto:martin.kallnik@gmx.de">Martin Kallnik</a>
* \author <a href="mailto:thomas.kindler@gmx.de">Thomas Kindler</a>
* \author Max Risler
*/

#pragma once

#include "Matrix3x3.h"

/**
 * Representation for 3x3 RotationMatrices
 */
class RotationMatrix : public Matrix3x3<> {
public:
    /**
     * Default constructor.
     */
    RotationMatrix() = default;

    RotationMatrix(
            const Vector3<> &c0,
            const Vector3<> &c1,
            const Vector3<> &c2) : Matrix3x3<>(c0, c1, c2) { }

    explicit RotationMatrix(const Matrix3x3<> &other) : Matrix3x3<>(other) { }

    RotationMatrix &operator=(const Matrix3x3<> &other) {
        c0 = other.c0;
        c1 = other.c1;
        c2 = other.c2;
        return *this;
    }

    RotationMatrix(const Vector3<> &axis, float angle);

    RotationMatrix(const Vector3<> &axis);

    RotationMatrix invert() const {
        return RotationMatrix(
                Vector3<>(c0.x, c1.x, c2.x),
                Vector3<>(c0.y, c1.y, c2.y),
                Vector3<>(c0.z, c1.z, c2.z)
        );
    }

    RotationMatrix operator*(const Matrix3x3<> &other) const {
        RotationMatrix result;
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

    Vector3<> operator*(const Vector3<> &vector) const {
        return Vector3<>(
                c0.x * vector.x + c1.x * vector.y + c2.x * vector.z,
                c0.y * vector.x + c1.y * vector.y + c2.y * vector.z,
                c0.z * vector.x + c1.z * vector.y + c2.z * vector.z);
    }

    RotationMatrix &rotateX(const float angle);

    RotationMatrix &rotateY(const float angle);

    RotationMatrix &rotateZ(const float angle);

    float getXAngle() const;

    float getYAngle() const;

    float getZAngle() const;

    static RotationMatrix fromRotationX(const float angle);

    static RotationMatrix fromRotationY(const float angle);

    static RotationMatrix fromRotationZ(const float angle);

    Vector3<> getAngleAxis() const;
};
