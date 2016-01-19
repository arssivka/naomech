#ifndef NAOMECH_REMOTECAMERA_H
#define NAOMECH_REMOTECAMERA_H

/*!
   \defgroup remote_camera RemoteCamera
   \ingroup remote_hardware
 */

#include <rd/network/RemoteModule.h>
#include <rd/hardware/Camera.h>

namespace rd {
///@{
/*!
   \brief Class for recieving the image from the cameras remotly
   This class allows to recieve nao's cameras images remotly via
   usage of RPC. This class is a child class of RemoteModule.
   The name of the module is cameras.
 */
    class RemoteCamera : public RemoteModule {
    public:
        /*!
           \brief Constructor for creating the RemoteCamera module
           \param top_camera shared pointer to the Camera object responsible for the working of the top nao's camera
           \param bot_camera shared pointer to the Camera object responsible for the working of the bottom nao's camera
         */
        RemoteCamera(boost::shared_ptr<rd::Camera> top_camera, boost::shared_ptr<rd::Camera> bot_camera);
    private:
        class ImageMethod : public RemoteMethod {
        public:
            /*!
               \brief Constructor for creating the remote ImageMethod
               \param top_camera shared pointer to the Camera object responsible for the working of the top nao's camera
               \param bot_camera shared pointer to the Camera object responsible for the working of the bottom nao's camera

                ImageMethod is the class, object of which is stored in the RemoteCamera module and represents the remote
                method named image, that you can call under the name proxy.cameras.image(). ImageMethod is the child class of
                the RemoteMethod and needed for recieving the images from cameras
             */
            ImageMethod(boost::shared_ptr<rd::Camera> top_camera, boost::shared_ptr<rd::Camera> bot_camera);

            /*!
               \brief This is the function that executes all the needed
               operations of the remote ImageMethod
               \param paramList boolean value. True means that image must be recieved from the top camera, otherwise from the bottom camera.
               \param resultP there will be stored returning value: structure that stores raw binary data buffer and timestamp
             */
            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

            ~ImageMethod();
        ///@}
        private:
            boost::shared_ptr<rd::Camera> top_camera;
            boost::shared_ptr<rd::Camera> bot_camera;
            xmlrpc_env envP;
            xmlrpc_value *valP;
        };
    };
}

#endif //NAOMECH_REMOTECAMERA_H
