#include <bhuman/CameraMatrix.h>
#include <bhuman/Transformation.h>

CameraMatrix::CameraMatrix(const Pose3D &pose)
        : Pose3D(pose),
          isValid(true) { }

CameraMatrix::CameraMatrix(const Pose3D &torsoMatrix,
                           const Pose3D &robotCameraMatrix,
                           const CameraCalibration &cameraCalibration) {
    computeCameraMatrix(torsoMatrix, robotCameraMatrix, cameraCalibration);
}

void CameraMatrix::computeCameraMatrix(const Pose3D &torsoMatrix,
                                       const Pose3D &robotCameraMatrix,
                                       const CameraCalibration &cameraCalibration) {
    (Pose3D&) *this = torsoMatrix;
    translate(cameraCalibration.bodyTranslationCorrection);
    rotateY(cameraCalibration.bodyTiltCorrection);
    rotateX(cameraCalibration.bodyRollCorrection);
    conc(robotCameraMatrix);
}
