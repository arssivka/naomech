//
// Created by arssivka on 10/5/15.
//

#ifndef NAOMECH_HARDWAREACCESSMODULE_H
#define NAOMECH_HARDWAREACCESSMODULE_H

#include "V4LRobotCamera.h"
#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>

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
        void setJointValues(const AL::ALValue &values);

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


    enum SensorType {
        HEAD_PITCH, HEAD_YAW,
        L_ANKLE_PITCH,
        L_ANKLE_ROLL,
        L_ELBOW_ROLL,
        L_ELBOW_YAW,
        L_HAND,
        L_HIP_PITCH,
        L_HIP_ROLL,
        L_HIP_YAW_PITCH,
        L_KNEE_PITCH,
        L_SHOULDER_PITCH,
        L_SHOULDER_ROLL,
        L_WRIST_YAW,
        R_ANKLE_PITCH,
        R_ANKLE_ROLL,
        R_ELBOW_ROLL,
        R_ELBOW_YAW,
        R_HAND,
        R_HIP_PITCH,
        R_HIP_ROLL,
        R_KNEE_PITCH,
        R_SHOULDER_PITCH,
        R_SHOULDER_ROLL,
        R_WRIST_YAW,
        ACC_X,
        ACC_Y,
        ACC_Z,
        GYR_X,
        GYR_Y,
        ANGLE_X,
        ANGLE_Y,
        L_COP_X,
        L_COP_Y,
        L_TOTAL_WEIGHT,
        R_COP_X,
        R_COP_Y,
        R_TOTAL_WEIGHT,
        SENSORS_COUNT
    };
}

#endif //NAOMECH_HARDWAREACCESSMODULE_H
