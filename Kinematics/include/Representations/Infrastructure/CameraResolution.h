#pragma once

class CameraInfo;

class CameraResolution {
public:
    enum Resolutions {
        defaultRes,

        upper640,
        lower640,
        both320,
        both640,
        noRequest
    };

    Resolutions resolution = defaultRes;
    unsigned timestamp = 0;

};