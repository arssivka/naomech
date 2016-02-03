#pragma once

#include <bhuman/tools/math/Pose3D.h>
#include <bhuman/representations/configuration/RobotDimensions.h>
#include <bhuman/representations/configuration/CameraCalibration.h>

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
