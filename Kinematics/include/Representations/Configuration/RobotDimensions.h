/**
* @file RobotDimensions.h
* Description of the Dimensions of the NAO Robot
* @author Cord Niehaus
*/

#pragma once

#include "Tools/Math/Vector3.h"

class RobotDimensions {
public:
    float getXHeadTiltToCamera(bool topCamera) const {
        return topCamera ? xHeadTiltToUpperCamera : xHeadTiltToCamera;
    }

    float getZHeadTiltToCamera(bool topCamera) const {
        return topCamera ? zHeadTiltToUpperCamera : zHeadTiltToCamera;
    }

    float getHeadTiltToCameraTilt(bool topCamera) const {
        return topCamera ? headTiltToUpperCameraTilt : headTiltToCameraTilt;
    }

    float lengthBetweenLegs;
    float upperLegLength;
    float lowerLegLength;
    float heightLeg5Joint;

    float zLegJoint1ToHeadPan;
    float xHeadTiltToCamera;
    float zHeadTiltToCamera;
    float headTiltToCameraTilt;

    float xHeadTiltToUpperCamera;
    float zHeadTiltToUpperCamera;
    float headTiltToUpperCameraTilt;

    Vector3<> armOffset;
    float yElbowShoulder;
    float upperArmLength;
    float lowerArmLength;
};
