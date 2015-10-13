#ifndef V4LROBOTCAMERA_H
#define V4LROBOTCAMERA_H

#include <string.h>
#include <iostream>
#include <errno.h>
#include <malloc.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

struct buffer {
        void* start;
        size_t length;
};

class V4LRobotCamera {
public:
    V4LRobotCamera(const char* device, int width, int height, bool blocking_mode);
    bool isCapturing();
    void startCapturing();
    void stopCapturing();
    cv::Mat getCVImage();
        cv::Mat getCRI();
    void setFPS(int rate);
	void autoWhiteBalance();
        unsigned char* captureImage();
	bool isStartOk();
	std::vector<unsigned char> getBinaryVector();
        ~V4LRobotCamera();

private:
    void initFMT();
    void initMMAP();
    int xioctl(int fd, int request, void* arg);
        static const unsigned int PIXEL_SIZE_YUV422 = 2;
    int w, h;
    int fd;
        struct buffer* tbuf;
        int size;
    bool is_capturing;
	bool startedNormally;
};

#endif //V4LROBOTCAMERA_H

