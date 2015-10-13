//
// Created by TekatoD on 29.09.2015.
//

#include "RD/HardwareAccessModule/V4LRobotCamera.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
using namespace std;

V4LRobotCamera::V4LRobotCamera(const char* device, int width, int height, bool blocking_mode) {
    w = width;
    h = height;
    if(blocking_mode) {
        fd = open(device, O_RDWR, 0);
        cout << "Blocking mode enabled" << endl;
    }
    else {
        fd = open(device, O_RDWR | O_NONBLOCK, 0);
        cout << "Non blocking mode" << endl;
    }
	startedNormally = true;
    if(fd == -1) {
        perror("Error opening video device ");
	startedNormally = false;
    }
    this->initFMT();
    this->initMMAP();
	this->autoWhiteBalance();
}

////////////////////////////////////////////////////////////////////////////////

bool V4LRobotCamera::isStartOk() {
	return startedNormally;
}

////////////////////////////////////////////////////////////////////////////////

void V4LRobotCamera::initFMT() {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof (struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = w;
    fmt.fmt.pix.height = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    int err = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if(err == -1) {
        perror("Setting format");
    }
}

////////////////////////////////////////////////////////////////////////////////

void V4LRobotCamera::autoWhiteBalance() {
	struct v4l2_queryctrl queryctrl;
	queryctrl.id = V4L2_CID_DO_WHITE_BALANCE;
	int err = 0;
	err = ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl);
	if(err == -1) {
		perror("Error querying ctrl");
	}
	struct v4l2_control control_s;
	control_s.id = V4L2_CID_DO_WHITE_BALANCE;
	control_s.value = 1;
	err = ioctl(fd, VIDIOC_S_CTRL, &control_s);
	if(err == -1) {
		perror("Error setting auto white balance");
	}
	
}

////////////////////////////////////////////////////////////////////////////////

void V4LRobotCamera::initMMAP() {
    int err = 0;
    struct v4l2_requestbuffers req;
    memset(&(req), 0, sizeof (v4l2_requestbuffers));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    err = ioctl(fd, VIDIOC_REQBUFS, &req);
    if(err == -1) {
        perror("Error requesting buffer");
    }
	tbuf = (struct buffer*) calloc(req.count, sizeof(*tbuf));
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof (struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    err = ioctl(fd, VIDIOC_QUERYBUF, &buf);
    if(err == -1) {
        perror("Error query buffer");
    }
	tbuf->length = buf.length;
	tbuf->start = mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
}

////////////////////////////////////////////////////////////////////////////////

void V4LRobotCamera::startCapturing() {
    int err = 0;
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof (struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.index = 0;
    buf.memory = V4L2_MEMORY_MMAP;
    err = xioctl(fd, VIDIOC_QUERYBUF, &buf);
    if(err == -1) {
        perror("Error quering buffer");
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(fd, VIDIOC_STREAMON, &type);
    if(err == -1) {
        perror("Error starting capture");
    }
    is_capturing = true;
}

////////////////////////////////////////////////////////////////////////////////

void V4LRobotCamera::stopCapturing() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int err = ioctl(fd, VIDIOC_STREAMOFF, &type);
    if(err == -1) {
        perror("Error stoping capture");
    }
    is_capturing = false;
}

////////////////////////////////////////////////////////////////////////////////

void V4LRobotCamera::setFPS(int rate) {
    int err = 0;
    struct v4l2_streamparm fps;
    memset(&fps, 0, sizeof (struct v4l2_streamparm));
    fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(fd, VIDIOC_G_PARM, &fps);
    if(err == -1) {
        perror("Error VIDIOC_G_PARAM ");
    }
    fps.parm.capture.timeperframe.numerator = 1;
    fps.parm.capture.timeperframe.denominator = rate;
    err = ioctl(fd, VIDIOC_S_PARM, &fps);
    if(err == -1) {
        perror("Error setting fps");
    }
}

////////////////////////////////////////////////////////////////////////////////

unsigned char * V4LRobotCamera::captureImage() {
    int err = 0;
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof (struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    err = xioctl(fd, VIDIOC_QBUF, &buf);
    if(err == -1) {
        perror("Error QBUF ");
    }
    err = xioctl(fd, VIDIOC_DQBUF, &buf);
    if(err == -1) {
        perror("Error retrieving image ");
    }
	size = buf.bytesused;
    return (unsigned char*)tbuf->start;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<unsigned char> V4LRobotCamera::getBinaryVector() {
	unsigned char* dbuf = this -> captureImage();
	std::vector<unsigned char> bin(dbuf, dbuf + size);
	return bin;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat V4LRobotCamera::getCVImage() {
    this -> captureImage();
	cv::Mat decoded = cv::Mat(h, w, CV_8UC3);
	unsigned char* dbuf = (unsigned char*) tbuf->start;
	for(int i = 0; i < w; ++i) {
		for(int j = 0; j < h; ++j) {
			double y = (double)dbuf[PIXEL_SIZE_YUV422 * (j * w + i)];
			double u = (double)dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 1 - ((i & 1)<<1)];
			double v = (double)dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 3 - ((i & 1)<<1)];
				
			decoded.at<cv::Vec3b>(j,i)[0] = (unsigned char)y;
			decoded.at<cv::Vec3b>(j,i)[1] = (unsigned char)u;
			decoded.at<cv::Vec3b>(j,i)[2] = (unsigned char)v;
 
		}
	
	}
    return decoded;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat V4LRobotCamera::getCRI() {
    this -> captureImage();
        cv::Mat decoded = cv::Mat(h, w, CV_8UC3);
        unsigned char* dbuf = (unsigned char*) tbuf->start;
        for(int i = 0; i < w; ++i) {
                for(int j = 0; j < h; ++j) {
                        double y = (double)dbuf[PIXEL_SIZE_YUV422 * (j * w + i)];
                        double u = (double)dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 1 - ((i & 1)<<1)];
                        double v = (double)dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 3 - ((i & 1)<<1)];
                        double r = y + 1.402 * (v - 128);
                        double g = y - 0.344 * (u -128) - 0.714 * (v -128);
                        double b = y + 1.772 * (u - 128);
                        if(r < 0.0) {
                                r = 0;
                        }
                        if(r > 255.0) {
                                r = 255.0;
                        }
                        if(g < 0.0) {
                                g = 0.0;
                        }
                        if(g > 255.0) {
                                g = 255.0;
                        }
                        if(b < 0.0) {
                                b = 0.0;
                        }
                        if(b > 255.0) {
                                b = 255.0;
                        }

                        decoded.at<cv::Vec3b>(j,i)[0] = (unsigned char)b;
                        decoded.at<cv::Vec3b>(j,i)[1] = (unsigned char)g;
                        decoded.at<cv::Vec3b>(j,i)[2] = (unsigned char)r;

                }

        }
//	 imwrite("/home/nao/captured/test.png", decoded);
    return decoded;
}

////////////////////////////////////////////////////////////////////////////////

int V4LRobotCamera::xioctl(int fd, int request, void* arg) {
    int r;
    do {
        r = ioctl (fd, request, arg);
    }
    while (-1 == r && EINTR == errno); // repeat if the call was interrupted
    return r;
}

////////////////////////////////////////////////////////////////////////////////

V4LRobotCamera::~V4LRobotCamera() {
	this -> stopCapturing();
	delete tbuf;
	
}
