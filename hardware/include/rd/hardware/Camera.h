//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_CAMERA_H
#define NAOMECH_CAMERA_H

#include "opencv2/core/core.hpp"
#include <rd/hardware/Image.h>
#include <rd/hardware/CvImage.h>
#include <boost/shared_ptr.hpp>


namespace rd {

    class Camera {
        public:
            Camera(const char *device, int width, int height,
                           bool blocking_mode);

            bool isCapturing();

            void startCapturing();

            void stopCapturing();

            boost::shared_ptr<rd::Image> getBinary();

            boost::shared_ptr<rd::CvImage> getCV();

            cv::Mat getCVImage();

            cv::Mat getCRI();

            void setFPS(int rate);

            void autoWhiteBalance();

            unsigned char *captureImage();

            bool isStartOk();

            int getSize();

            std::vector<unsigned char> getBinaryVector();

            ~Camera();

        private:
            struct VideoImageBuffer {
                    void *start;
                    size_t length;
            };

            void initFMT();

            void initMMAP();

            int xioctl(int fd, int request, void *arg);

            static const unsigned int PIXEL_SIZE_YUV422 = 2;
            int w, h;
            int fd;
            struct VideoImageBuffer *tbuf;
            int size;
            bool is_capturing;
            bool startedNormally;
        };
}

#endif //NAOMECH_CAMERA_H
