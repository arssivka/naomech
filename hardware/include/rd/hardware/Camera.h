#ifndef NAOMECH_CAMERA_H
#define NAOMECH_CAMERA_H

#include "opencv2/core/core.hpp"
#include <rd/representation/CvImage.h>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <alcommon/albroker.h>
#include <alproxies/dcmproxy.h>


/*!        \defgroup camera Camera
           \ingroup hardware
 */


namespace rd {
///@{
    /*!
       \brief Class that provides the working of the specified camera of nao
       via usage of video4linux library
     */
    class Camera : boost::noncopyable {
        public:
            /*!
               \brief Constructor for creating the camera object
               \param device name of the device that must be used. "/dev/video0" or "/dev/video1" in nao case
               \param width width of the images that must be taken from the camera
               \param height height of the images that must be taken from the camera
               \param blocking_mode if true runs the camera in the blocking mode, otherwise ninblocking mode
               \param broker broker that allows to connect to the nao's modules
             */
            Camera(const char *device, int width, int height,
                           bool blocking_mode, boost::shared_ptr<AL::ALBroker> broker);

            /*!
               \brief Cheks if camera is enabled
               \return true if camera is enabled, otherwise false
             */
            bool isEnabled();

            /*!
               \brief Turns on the camera
             */
            void enableCamera();

            /*!
               \brief Turns of the camera
             */
            void disableCamera();

            /*!
               \brief Allows to recieve image in opencv format and timestamp. YUV colorspace.
               \return shared pointer to the CvImage class
             */
            boost::shared_ptr<rd::CvImage> getCV();

            /*!
               \brief Allows to recieve image in opencv format. YUV colorspace.
               \return cv::Mat image
             */
            cv::Mat getCVImage();

            /*!
               \brief Allows to recieve image in opencv format. RGB colorspace.
               \return cv::Mat image
             */
            cv::Mat getCRI();

            /*!
               \brief Sets the camera framerate
               \param rate desired framerate
             */
            void setFPS(int rate);

            /*!
               \brief runs auto white balance on the camera
             */
            void autoWhiteBalance();

            /*!
               \brief Captures the image. Raw buffer.
               \return Raw buffer of image data
             */
            unsigned char *captureImage();

            /*!
               \brief Checks if camera was started normally
               \return true if camera was started normalle, otherwise false
             */
            bool isStartOk();

            /*!
               \brief Returns the size of the buffer that was used for capturing the image last time
               \return
             */
            int getSize();

            /*!
               \brief Allows to recieve the current timestamp
               \return timestamp
             */
            int getTime();

            /*!
               \brief Allows to recieve std vector of the raw binary data
               \return std vector of unsigned char
             */
            std::vector<unsigned char> getBinaryVector();

            /*!
                \brief Destructor
             */
            ~Camera();
///@}
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

            boost::shared_ptr<AL::DCMProxy> dcm;
        };
}

#endif //NAOMECH_CAMERA_H
