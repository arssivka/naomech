#pragma once

#include <bhuman/JointData.h>
#include <bhuman/BHMath.h>

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

    JointInfo joints[rd::Joints::JOINTS_COUNT];
};
