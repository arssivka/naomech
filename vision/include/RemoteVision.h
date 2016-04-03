//
// Created by nikitas on 4/1/16.
//

#ifndef NAOMECH_REMOTEVISION_H
#define NAOMECH_REMOTEVISION_H

#include <boost/shared_ptr.hpp>

#include <rd/hardware/Camera.h>

#include <rd/network/RemoteMethod.h>
#include <rd/network/RemoteModule.h>

#include "Vision.h"

namespace rd {

    class RemoteVision : public RemoteModule {
    public:
        RemoteVision(boost::shared_ptr<Vision> vision, boost::shared_ptr<Camera> camera);

    private:
        class ballDetect : public RemoteMethod {
        public:
            ballDetect(boost::shared_ptr<Vision> vision);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Vision> m_vision;
        };

        class lineDetect : public RemoteMethod {
        public:
            lineDetect(boost::shared_ptr<Vision> vision);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Vision> m_vision;
        };

        class updateFrame : public RemoteMethod {
        public:
            updateFrame(boost::shared_ptr<Vision> vision, boost::shared_ptr<Camera> camera);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Vision> m_vision;
            boost::shared_ptr<Camera> m_camera;
        };
    };


}


#endif //NAOMECH_REMOTEVISION_H
