//
// Created by arssivka on 12/15/15.
//

#ifndef NAOMECH_HWCONTROLLER_H
#define NAOMECH_HWCONTROLLER_H

#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>


namespace AL {
    class ALBroker;
    class ALMemoryFastAccess;
    class DCMProxy;
}

namespace rd {
    enum SensorType {
        HEAD_YAW, HEAD_PITCH,
        L_SHOULDER_PITCH,
        L_SHOULDER_ROLL,
        L_ELBOW_YAW,
        L_ELBOW_ROLL,
        L_WRIST_YAW,
        L_HAND,
        L_HIP_YAW_PITCH,
        L_HIP_ROLL,
        L_HIP_PITCH,
        L_KNEE_PITCH,
        L_ANKLE_PITCH,
        L_ANKLE_ROLL,
        R_HIP_YAW_PITCH,
        R_HIP_ROLL,
        R_HIP_PITCH,
        R_KNEE_PITCH,
        R_ANKLE_PITCH,
        R_ANKLE_ROLL,
        R_SHOULDER_PITCH,
        R_SHOULDER_ROLL,
        R_ELBOW_YAW,
        R_ELBOW_ROLL,
        R_WRIST_YAW,
        R_HAND,
        NUM_OF_JOINTS
    };

    class HWController : public AL::ALModule {
    public:
        HWController(boost::shared_ptr<AL::ALBroker> pBroker,
                     const std::string &pName);

        virtual ~HWController();

        virtual void init();

        void setJointData(const AL::ALValue &values);

        AL::ALValue getJointData();

    private:

        void startLoop();

        void stopLoop();

        void initFastAccess();

        void connectToDCMloop();

        void synchronisedDCMcallback();

        void createPositionActuatorAlias();

        void preparePositionActuatorCommand();

        ProcessSignalConnection dcm_post_process_connection;
        boost::shared_ptr<AL::ALMemoryFastAccess> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;

        std::vector<std::string> sensor_keys;

        std::vector<float> sensor_data;
        AL::ALValue sensor_values;

        boost::mutex actuator_mutex;
        boost::mutex sensor_mutex;

        boost::shared_ptr<std::vector<float> > work_actuator_values;
        AL::ALValue commands;
    };
}

#endif //NAOMECH_HWCONTROLLER_H
