#ifndef NAOMECH_CVIMAGE_H
#define NAOMECH_CVIMAGE_H

#include "opencv2/core/core.hpp"

namespace rd {
    class CvImage {
    public:
        CvImage() {}
        CvImage(cv::Mat img, int timestamp): img(img), timestamp(timestamp) { }

        int timestamp;
        cv::Mat img;
    };
}

#endif //NAOMECH_CVIMAGE_H
