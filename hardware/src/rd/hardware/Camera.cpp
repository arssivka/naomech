#include "rd/hardware/Camera.h"
#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

using namespace std;
using namespace rd;
using namespace boost;
using namespace AL;

Camera::Camera(const char *device, int width, int height,
               bool blocking_mode,
               shared_ptr<ALBroker> broker) : m_dcm(make_shared<DCMProxy>(broker)) {
    m_w = width;
    m_h = height;
    if (blocking_mode) {
        m_fd = open(device, O_RDWR, 0);
    }
    else {
        m_fd = open(device, O_RDWR | O_NONBLOCK, 0);
    }
    m_started_normally = true;
    if (m_fd == -1) {
        perror("Error opening video device ");
        m_started_normally = false;
    }
    this->initFMT();
    this->initMMAP();
//    this->autoWhiteBalance();
}

////////////////////////////////////////////////////////////////////////////////

bool Camera::isStartOk() {
    return m_started_normally;
}

////////////////////////////////////////////////////////////////////////////////

void Camera::initFMT() {
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(struct v4l2_format));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = m_w;
    fmt.fmt.pix.height = m_h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    int err = ioctl(m_fd, VIDIOC_S_FMT, &fmt);
    if (err == -1) {
        perror("Setting format");
    }
}

////////////////////////////////////////////////////////////////////////////////

void Camera::autoWhiteBalance() {
    struct v4l2_queryctrl queryctrl;
    queryctrl.id = V4L2_CID_DO_WHITE_BALANCE;
    int err = 0;
    err = ioctl(m_fd, VIDIOC_QUERYCTRL, &queryctrl);
    if (err == -1) {
        perror("Error querying ctrl");
    }
    struct v4l2_control control_s;
    control_s.id = V4L2_CID_DO_WHITE_BALANCE;
    control_s.value = 1;
    err = ioctl(m_fd, VIDIOC_S_CTRL, &control_s);
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
    err = ioctl(m_fd, VIDIOC_REQBUFS, &req);
    if (err == -1) {
        perror("Error requesting buffer");
    }
    m_tbuf = (struct VideoImageBuffer*) calloc(req.count, sizeof(*m_tbuf));
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;
    err = ioctl(m_fd, VIDIOC_QUERYBUF, &buf);
    if (err == -1) {
        perror("Error query buffer");
    }
    m_tbuf->length = buf.length;
    m_tbuf->start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, m_fd,
                         buf.m.offset);
}

////////////////////////////////////////////////////////////////////////////////

int Camera::getTime() {
    return this->m_dcm->getTime(0);
}

////////////////////////////////////////////////////////////////////////////////

void Camera::enableCamera() {
    int err = 0;
    struct v4l2_buffer buf;
    memset(&(buf), 0, sizeof(struct v4l2_buffer));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.index = 0;
    buf.memory = V4L2_MEMORY_MMAP;
    err = xioctl(m_fd, VIDIOC_QUERYBUF, &buf);
    if (err == -1) {
        perror("Error quering buffer");
    }
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(m_fd, VIDIOC_STREAMON, &type);
    if (err == -1) {
        perror("Error starting capture");
    }
    m_is_capturing = true;
}

////////////////////////////////////////////////////////////////////////////////

void Camera::disableCamera() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int err = ioctl(m_fd, VIDIOC_STREAMOFF, &type);
    if (err == -1) {
        perror("Error stoping capture");
    }
    m_is_capturing = false;
}

////////////////////////////////////////////////////////////////////////////////

bool Camera::isEnabled() {
    return m_is_capturing;
}

///////////////////////////////////////////////////////////////////////////////////

void Camera::setFPS(int rate) {
    int err = 0;
    struct v4l2_streamparm fps;
    memset(&fps, 0, sizeof(struct v4l2_streamparm));
    fps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    err = ioctl(m_fd, VIDIOC_G_PARM, &fps);
    if (err == -1) {
        perror("Error VIDIOC_G_PARAM ");
    }
    fps.parm.capture.timeperframe.numerator = 1;
    fps.parm.capture.timeperframe.denominator = rate;
    err = ioctl(m_fd, VIDIOC_S_PARM, &fps);
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
    err = xioctl(m_fd, VIDIOC_QBUF, &buf);
    if (err == -1) {
        perror("Error QBUF ");
    }
    err = xioctl(m_fd, VIDIOC_DQBUF, &buf);
    if (err == -1) {
        perror("Error retrieving image ");
    }
    m_size = buf.bytesused;
    return (unsigned char*) m_tbuf->start;
}

////////////////////////////////////////////////////////////////////////////////

shared_ptr<CvImage> Camera::getCV() {
    this->captureImage();
    cv::Mat decoded = cv::Mat(m_h, m_w, CV_8UC3);
    unsigned char* dbuf = (unsigned char*) m_tbuf->start;
    for (int i = 0; i < m_w; ++i) {
        for (int j = 0; j < m_h; ++j) {
            int temp = (j * m_w + i) * PIXEL_SIZE_YUV422;
            double y = (double) dbuf[(temp)];
            double u = (double) dbuf[(temp) + 1 -
                                     ((i & 1) << 1)];
            double v = (double) dbuf[(temp) + 3 -
                                     ((i & 1) << 1)];

            decoded.at<cv::Vec3b>(j, i)[0] = (unsigned char) y;
            decoded.at<cv::Vec3b>(j, i)[1] = (unsigned char) u;
            decoded.at<cv::Vec3b>(j, i)[2] = (unsigned char) v;

        }
    }
    return make_shared<CvImage>(decoded, this->m_dcm->getTime(0));
}

////////////////////////////////////////////////////////////////////////////////

vector<unsigned char> Camera::getBinaryVector() {
    unsigned char *dbuf = this->captureImage();
    vector<unsigned char> bin(dbuf, dbuf + m_size);
    return bin;
}

////////////////////////////////////////////////////////////////////////////////

int Camera::getSize() {
    return m_size;
}

////////////////////////////////////////////////////////////////////////////////

cv::Mat Camera::getCVImage() {
    this->captureImage();
    cv::Mat decoded = cv::Mat(m_h, m_w, CV_8UC3);
    unsigned char* dbuf = (unsigned char*) m_tbuf->start;
    for (int i = 0; i < m_w; ++i) {
        for (int j = 0; j < m_h; ++j) {
            double y = (double) dbuf[PIXEL_SIZE_YUV422 * (j * m_w + i)];
            double u = (double) dbuf[PIXEL_SIZE_YUV422 * (j * m_w + i) + 1 -
                                     ((i & 1) << 1)];
            double v = (double) dbuf[PIXEL_SIZE_YUV422 * (j * m_w + i) + 3 -
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
    cv::Mat decoded = cv::Mat(m_h, m_w, CV_8UC3);
    unsigned char* dbuf = (unsigned char*) m_tbuf->start;
    for (int i = 0; i < m_w; ++i) {
        for (int j = 0; j < m_h; ++j) {
            double y = (double) dbuf[PIXEL_SIZE_YUV422 * (j * m_w + i)];
            double u = (double) dbuf[PIXEL_SIZE_YUV422 * (j * m_w + i) + 1 -
                                     ((i & 1) << 1)];
            double v = (double) dbuf[PIXEL_SIZE_YUV422 * (j * m_w + i) + 3 -
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
    delete m_tbuf;

}
