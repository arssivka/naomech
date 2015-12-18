#ifndef NAOMECH_IMAGE_H
#define NAOMECH_IMAGE_H

#include <boost/shared_array.hpp>
#include "opencv2/core/core.hpp"

namespace rd {
    class Image {
    public:
        Image() {}
        Image(unsigned char* bin, int timestamp): timestamp(timestamp) {
            this->data.bin = boost::shared_array<unsigned char>(bin);
        }
        Image(cv::Mat img, int timestamp): timestamp(timestamp) {
            this->data.cv_image = img;
        }
        int timestamp;
        imagedata data;
        union imagedata{
            boost::shared_array<unsigned char> bin;
            cv::Mat cv_image;
        };
    };
}

#endif //NAOMECH_IMAGE_H
