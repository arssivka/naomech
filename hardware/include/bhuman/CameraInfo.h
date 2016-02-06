#pragma once

#include <bhuman/Vector2.h>

class CameraInfo {
public:
    enum Camera {
        upper,
        lower
    };

    Camera camera;
    int width;
    int height;
    float openingAngleWidth;
    float openingAngleHeight;
    Vector2<> opticalCenter;

    float focalLength;
    float focalLengthInv;
    float focalLenPow2;

    CameraInfo() = default;

    CameraInfo(Camera camera) :
            camera(camera) { }

    void updateFocalLength();
};

class CameraInfoFullRes : public CameraInfo {
public:
    CameraInfoFullRes() = default;

    explicit CameraInfoFullRes(const CameraInfo &other) :
            CameraInfo(other) {
        width *= 2;
        height *= 2;
        opticalCenter *= 2.f;
        updateFocalLength();
    };

    CameraInfoFullRes &operator=(const CameraInfo &other) {
        camera = other.camera;
        width = other.width * 2;
        height = other.height * 2;
        openingAngleWidth = other.openingAngleWidth;
        openingAngleHeight = other.openingAngleHeight;
        opticalCenter = other.opticalCenter * 2.f;
        updateFocalLength();
        return *this;
    };
};
