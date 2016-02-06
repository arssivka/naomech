/**
* @file Tools/Math/Transformation.h
*
* Declares a class that contains a set of static methods for
* coordinate system transformations.
*
* @author <a href="mailto:tlaue@uni-bremen.de.de">Tim Laue</a>
*/

#pragma once

#include <bhuman/Pose2D.h>
#include <bhuman/Vector3.h>

class CameraMatrix;

class CameraInfo;


/**
* The class Transformation defines methods for
* coordinate system transformations
*/
class Transformation {
public:
    static Vector2<> robotToField(const Pose2D &rp, const Vector2<> &relPos);

    static Vector2<> fieldToRobot(const Pose2D &rp, const Vector2<> &fieldPos);

    static bool imageToRobot(const float x,
                             const float y,
                             const CameraMatrix &cameraMatrix,
                             const CameraInfo &cameraInfo,
                             Vector2<> &relativePosition);

    static bool imageToRobot(const int x,
                             const int y,
                             const CameraMatrix &cameraMatrix,
                             const CameraInfo &cameraInfo,
                             Vector2<> &relativePosition);

    static bool imageToRobot(const int x,
                             const int y,
                             const CameraMatrix &cameraMatrix,
                             const CameraInfo &cameraInfo,
                             Vector2<int> &relativePosition);

    static bool imageToRobot(const Vector2<int> &pointInImage,
                             const CameraMatrix &cameraMatrix,
                             const CameraInfo &cameraInfo,
                             Vector2<> &relativePosition);

    static bool imageToRobotHorizontalPlane(const Vector2<> &pointInImage,
                                            float z,
                                            const CameraMatrix &cameraMatrix,
                                            const CameraInfo &cameraInfo,
                                            Vector2<> &pointOnPlane);

    static bool robotToImage(const Vector2<> &point,
                             const CameraMatrix &cameraMatrix,
                             const CameraInfo &cameraInfo,
                             Vector2<> &pointInImage);

    static bool robotToImage(const Vector3<> &point,
                             const CameraMatrix &cameraMatrix,
                             const CameraInfo &cameraInfo,
                             Vector2<> &pointInImage);

    static bool robotWithCameraRotationToImage(const Vector2<> &point,
                                               const CameraMatrix &cameraMatrix,
                                               const CameraInfo &cameraInfo,
                                               Vector2<> &pointInImage);

    static bool imageToRobotWithCameraRotation(const Vector2<int> &pointInImage,
                                               const CameraMatrix &cameraMatrix,
                                               const CameraInfo &cameraInfo,
                                               Vector2<> &relativePosition);
};
