/**
* @file Representations/Infrastructure/JointData.h
*
* This file declares a classes to represent the joint angles.
*
* @author <A href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</A>
*/

#pragma once

#include <cmath>

class JointData {
public:
    enum Joint {
        HeadYaw,
        HeadPitch,
        LShoulderPitch,
        LShoulderRoll,
        LElbowYaw,
        LElbowRoll,
        RShoulderPitch,
        RShoulderRoll,
        RElbowYaw,
        RElbowRoll,
        LHipYawPitch,
        LHipRoll,
        LHipPitch,
        LKneePitch,
        LAnklePitch,
        LAnkleRoll,
        RHipYawPitch,
        RHipRoll,
        RHipPitch,
        RKneePitch,
        RAnklePitch,
        RAnkleRoll,
        numOfJoints
    };

    enum {
        off = 1000
    };

    enum {
        ignore = 2000
    };

    JointData() {
        for (int i = 0; i < numOfJoints; ++i)
            angles[i] = off;
    }

    float angles[numOfJoints];
};