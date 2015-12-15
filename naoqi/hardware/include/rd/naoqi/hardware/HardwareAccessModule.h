//
// Created by arssivka on 12/15/15.
//

#ifndef NAOMECH_HARDWAREACCESSMODULE_H
#define NAOMECH_HARDWAREACCESSMODULE_H

#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include "RD/HardwareAccessModule/V4LRobotCamera.h"


namespace AL {
    class ALBroker;

    class ALMemoryFastAccess;

    class DCMProxy;
}

namespace RD {
    class HardwareAccessModule : public AL::ALModule {
    public:
        HardwareAccessModule(boost::shared_ptr<AL::ALBroker> pBroker,
                             const std::string &pName);

        virtual ~HardwareAccessModule();

        virtual void init();

        void stopStream();

        bool checkDevices();

        AL::ALValue getImageBufferTop();

        AL::ALValue getImageBufferBot();

    private:
        void setJoints(const AL::ALValue &values);

        void setStiffness(const float &stiffnessValue);

        AL::ALValue getSensorsValues();

        void startLoop();

        void stopLoop();

        void initFastAccess();

        void connectToDCMloop();

        void synchronisedDCMcallback();

        void createHardnessActuatorAlias();

        void createPositionActuatorAlias();

        void preparePositionActuatorCommand();

        void initDCM();

        ProcessSignalConnection dcm_post_process_connection;
        boost::shared_ptr<AL::ALMemoryFastAccess> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;

        std::vector<std::string> sensor_keys;

        std::vector<float> local_sensor_values;
        AL::ALValue sensor_values;

        boost::mutex actuator_mutex;

        boost::shared_ptr<std::vector<float> > work_actuator_values;
        AL::ALValue commands;

        boost::shared_ptr<V4LRobotCamera> top_camera;
        boost::shared_ptr<V4LRobotCamera> bottom_camera;
    };
}

#endif //NAOMECH_HARDWAREACCESSMODULE_H
