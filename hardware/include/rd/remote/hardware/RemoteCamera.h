#ifndef NAOMECH_REMOTECAMERA_H
#define NAOMECH_REMOTECAMERA_H

#include <rd/network/RemoteModule.h>
#include <rd/hardware/Camera.h>

namespace rd {
    class RemoteCamera : public RemoteModule {
    public:
        RemoteCamera(boost::shared_ptr<rd::Camera> top_camera, boost::shared_ptr<rd::Camera> bot_camera);
    private:
        class ImageMethod : public RemoteMethod {
        public:
            ImageMethod(boost::shared_ptr<rd::Camera> top_camera, boost::shared_ptr<rd::Camera> bot_camera);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

            ~ImageMethod();

        private:
            boost::shared_ptr<rd::Camera> top_camera;
            boost::shared_ptr<rd::Camera> bot_camera;
            xmlrpc_env envP;
            xmlrpc_value *valP;
        };
    };
}

#endif //NAOMECH_REMOTECAMERA_H
