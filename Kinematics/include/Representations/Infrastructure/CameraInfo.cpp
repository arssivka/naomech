#include "CameraInfo.h"

void CameraInfo::updateFocalLength() {
    focalLength = width / (2.f * std::tan(this->openingAngleWidth / 2.f));
    focalLengthInv = 1.0f / focalLength;
    focalLenPow2 = focalLength * focalLength;
}