#pragma once

#include "Tools/Math/Vector3.h"

class CameraCalibration {
public:
    float lowerCameraTiltCorrection;
    float lowerCameraRollCorrection;
    float lowerCameraPanCorrection;
    float upperCameraTiltCorrection;
    float upperCameraRollCorrection;
    float upperCameraPanCorrection;
    float bodyTiltCorrection;
    float bodyRollCorrection;
    Vector3<> bodyTranslationCorrection;
};