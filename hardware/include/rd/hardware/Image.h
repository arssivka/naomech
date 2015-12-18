#ifndef NAOMECH_IMAGE_H
#define NAOMECH_IMAGE_H

#include <boost/shared_array.hpp>

namespace rd {
    class Image {
    public:
        Image() {}
        Image(unsigned char* bin, int timestamp): timestamp(timestamp) {
            this->bin = boost::shared_array<unsigned char>(bin);
        }
        int timestamp;
        boost::shared_array<unsigned char> bin;
    };
}

#endif //NAOMECH_IMAGE_H
