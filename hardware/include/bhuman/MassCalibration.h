/**
* @file MassCalibration.h
* Declaration of a class for representing the relative positions and masses of mass points.
* @author <a href="mailto:allli@informatik.uni-bremen.de">Alexander Härtl</a>
*/

#pragma once

#include <bhuman/Vector3.h>

class MassCalibration {
public:
    enum Limb {
        neck,
        head,
        shoulderLeft,
        bicepsLeft,
        elbowLeft,
        foreArmLeft,
        shoulderRight,
        bicepsRight,
        elbowRight,
        foreArmRight,
        pelvisLeft,
        hipLeft,
        thighLeft,
        tibiaLeft,
        ankleLeft,
        footLeft,
        pelvisRight,
        hipRight,
        thighRight,
        tibiaRight,
        ankleRight,
        footRight,
        torso,
        numOfLimbs
    };

    class MassInfo {
    public:
        float mass;
        Vector3<> offset;
    };

    MassInfo masses[numOfLimbs];
};
