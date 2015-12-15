#pragma once

#include "Tools/Math/Pose3D.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"

class CameraMatrix : public Pose3D {
public:
    CameraMatrix() = default;

    CameraMatrix(const Pose3D &pose);

    CameraMatrix(const Pose3D &torsoMatrix, const Pose3D &robotCameraMatrix,
                 const CameraCalibration &cameraCalibration);

    void computeCameraMatrix(const Pose3D &torsoMatrix,
                             const Pose3D &robotCameraMatrix,
                             const CameraCalibration &cameraCalibration);

    bool isValid;
};
