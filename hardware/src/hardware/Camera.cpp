#include "rd/hardware/Camera.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <iostream>
#include <boost/make_shared.hpp>

using namespace std;
using namespace rd;
using namespace boost;
using namespace AL;

Camera::Camera(const char *device, int width, int height,
                               bool blocking_mode,
               shared_ptr<ALBroker> broker): dcm(make_shared<DCMProxy>(broker)) {
    w = width;
    h = height;
    if (blocking_mode) {
        fd = open(device, O_RDWR, 0);
    }
    else {
        fd = open(device, O_RDWR | O_NONBLOCK, 0);
    }
    startedNormally = true;
    if (fd == -1) {
        perror("Error opening video device ");
        startedNormally = false;
    }
    this->initFMT();
    this->initMMAP();
    this->autoWhiteBalance();
}

////////////////////////////////////////////////////////////////////////////////

bool Camera::isStartOk() {
    return startedNormally;
}

////////////////////////////////////////////////////////////////////////////////

void Camera::initFMT() {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = w;
    fmt.fmt.pix.height = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    int err = ioctl(fd, VIDIOC_S_FMT, &fmt);
    if (err == -1) {
        perror("Setting format");
    }
}

////////////////////////////////////////////////////////////////////////////////

void Camera::autoWhiteBalance() {
    struct v4l2_queryctrl queryctrl;
    queryctrl.id = V4L2_CID_DO_WHITE_BALANCE;
    int err = 0;
    err = ioctl(fd, VIDIOC_QUERYCTRL, &queryctrl);
    if (err == -1) {
        perror("Error querying ctrl");
    }
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_DO_WHITE_BALANCE;
    control_s.value = 1;
    err = ioctl(fd, VIDIOC_S_CTRL, &control_s);
    if (err == -1) {
        perror("Error setting auto white balance");
    }

}

////////////////////////////////////////////////////////////////////////////////

void Camera::initMMAP() {
    int err = 0;
    struct v4l2_requestbuffers req;
    memset(&(req), 0, sizeof(v4l2_requestbuffers));
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    err = ioctl(fd, VIDIOC_REQBUFS, &req);
    if (err == -1) {
        perror("Error requesting buffer");
    }
    tbuf = (struct VideoImageBuffer *) calloc(req.count, sizeof(*tbuf));
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    err = ioctl(fd, VIDIOC_QUERYBUF, &buf);
    if (err == -1) {
        perror("Error query buffer");
    }
    tbuf->length = buf.length;
    tbuf->start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                       buf.m.offset);
}

////////////////////////////////////////////////////////////////////////////////

void Camera::enableCamera() {
    int err = 0;
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.index = 0;
    buf.memory = V4L2_MEMORY_MMAP;
    err = xioctl(fd, VIDIOC_QUERYBUF, &buf);
    if (err == -1) {
        perror("Error quering buffer");
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(fd, VIDIOC_STREAMON, &type);
    if (err == -1) {
        perror("Error starting capture");
    }
    is_capturing = true;
}

////////////////////////////////////////////////////////////////////////////////

void Camera::disableCamera() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int err = ioctl(fd, VIDIOC_STREAMOFF, &type);
    if (err == -1) {
        perror("Error stoping capture");
    }
    is_capturing = false;
}

////////////////////////////////////////////////////////////////////////////////

bool Camera::isEnabled() {
    return is_capturing;
}

///////////////////////////////////////////////////////////////////////////////////

void Camera::setFPS(int rate) {
    int err = 0;
    struct v4l2_streamparm fps;
    memset(&fps, 0, sizeof(struct v4l2_streamparm));
    fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(fd, VIDIOC_G_PARM, &fps);
    if (err == -1) {
        perror("Error VIDIOC_G_PARAM ");
    }
    fps.parm.capture.timeperframe.numerator = 1;
    fps.parm.capture.timeperframe.denominator = rate;
    err = ioctl(fd, VIDIOC_S_PARM, &fps);
    if (err == -1) {
        perror("Error setting fps");
    }
}

////////////////////////////////////////////////////////////////////////////////

unsigned char *Camera::captureImage() {
    int err = 0;
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    err = xioctl(fd, VIDIOC_QBUF, &buf);
    if (err == -1) {
        perror("Error QBUF ");
    }
    err = xioctl(fd, VIDIOC_DQBUF, &buf);
    if (err == -1) {
        perror("Error retrieving image ");
    }
    size = buf.bytesused;
    return (unsigned char *) tbuf->start;
}

////////////////////////////////////////////////////////////////////////////////
shared_ptr<Image> Camera::getBinary() {
    unsigned char *dbuf = this->captureImage();
    return make_shared<Image>(dbuf, dcm->getTime(0));
}

////////////////////////////////////////////////////////////////////////////////

shared_ptr<CvImage> Camera::getCV() {
    this->captureImage();
    cv::Mat decoded = cv::Mat(h, w, CV_8UC3);
    unsigned char *dbuf = (unsigned char *) tbuf->start;
    for (int i = 0; i < w; ++i) {
        for (int j = 0; j < h; ++j) {
            int temp = j * w + i;
            double y = (double) dbuf[PIXEL_SIZE_YUV422 * (temp)];
            double u = (double) dbuf[PIXEL_SIZE_YUV422 * (temp) + 1 -
                                     ((i & 1) << 1)];
            double v = (double) dbuf[PIXEL_SIZE_YUV422 * (temp) + 3 -
                                     ((i & 1) << 1)];

            decoded.at<cv::Vec3b>(j, i)[0] = (unsigned char) y;
            decoded.at<cv::Vec3b>(j, i)[1] = (unsigned char) u;
            decoded.at<cv::Vec3b>(j, i)[2] = (unsigned char) v;

        }
    }
    //TODO: Get the time from dcm module
    return make_shared<CvImage>(decoded, 0);
}

////////////////////////////////////////////////////////////////////////////////

vector<unsigned char> Camera::getBinaryVector() {
    unsigned char *dbuf = this->captureImage();
    vector<unsigned char> bin(dbuf, dbuf + size);
    return bin;
}

////////////////////////////////////////////////////////////////////////////////

int Camera::getSize() {
    return this->size;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat Camera::getCVImage() {
    this->captureImage();
    cv::Mat decoded = cv::Mat(h, w, CV_8UC3);
    unsigned char *dbuf = (unsigned char *) tbuf->start;
    for (int i = 0; i < w; ++i) {
        for (int j = 0; j < h; ++j) {
            double y = (double) dbuf[PIXEL_SIZE_YUV422 * (j * w + i)];
            double u = (double) dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 1 -
                                     ((i & 1) << 1)];
            double v = (double) dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 3 -
                                     ((i & 1) << 1)];

            decoded.at<cv::Vec3b>(j, i)[0] = (unsigned char) y;
            decoded.at<cv::Vec3b>(j, i)[1] = (unsigned char) u;
            decoded.at<cv::Vec3b>(j, i)[2] = (unsigned char) v;

        }

    }
    return decoded;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat Camera::getCRI() {
    this->captureImage();
    cv::Mat decoded = cv::Mat(this->h, this->w, CV_8UC3);
    unsigned char *dbuf = (unsigned char *) tbuf->start;
    for (int i = 0; i < w; ++i) {
        for (int j = 0; j < h; ++j) {
            double y = (double) dbuf[PIXEL_SIZE_YUV422 * (j * w + i)];
            double u = (double) dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 1 -
                                     ((i & 1) << 1)];
            double v = (double) dbuf[PIXEL_SIZE_YUV422 * (j * w + i) + 3 -
                                     ((i & 1) << 1)];
            double r = y + 1.402 * (v - 128);
            double g = y - 0.344 * (u - 128) - 0.714 * (v - 128);
            double b = y + 1.772 * (u - 128);
            if (r < 0.0) {
                r = 0;
            }
            if (r > 255.0) {
                r = 255.0;
            }
            if (g < 0.0) {
                g = 0.0;
            }
            if (g > 255.0) {
                g = 255.0;
            }
            if (b < 0.0) {
                b = 0.0;
            }
            if (b > 255.0) {
                b = 255.0;
            }

            decoded.at<cv::Vec3b>(j, i)[0] = (unsigned char) b;
            decoded.at<cv::Vec3b>(j, i)[1] = (unsigned char) g;
            decoded.at<cv::Vec3b>(j, i)[2] = (unsigned char) r;

        }

    }
//	 imwrite("/home/nao/captured/test.png", decoded);
    return decoded;
}

////////////////////////////////////////////////////////////////////////////////

int Camera::xioctl(int fd, int request, void *arg) {
    int r;
    do {
        r = ioctl(fd, request, arg);
    }
    while (-1 == r && EINTR == errno); // repeat if the call was interrupted
    return r;
}

////////////////////////////////////////////////////////////////////////////////

Camera::~Camera() {
    this->disableCamera();
    delete tbuf;

}
