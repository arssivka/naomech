#pragma once

#include "Representations/Infrastructure/JointData.h"
#include "Tools/Math/BHMath.h"

class JointCalibration {
public:
    class JointInfo {

    public:
        float offset;
        short sign;
        float maxAngle;
        float minAngle;

        JointInfo() : offset(0), sign(1), maxAngle(2.618f),
                      minAngle(-2.618f) { }
    };

    JointInfo joints[JointData::numOfJoints];
};
