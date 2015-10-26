//
// Created by arssivka on 10/5/15.
//

#ifndef NAOMECH_KINEMATICSMODULE_H
#define NAOMECH_KINEMATICSMODULE_H

#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include "RD/KinematicsModule/NAOKinematics.h"
#include "RD/HardwareAccessModule/HardwareDefines.h"

namespace AL {
    class ALBroker;
    class ALMemoryFastAccess;
    class ALProxy;
}

namespace RD {
    class KinematicsModule : public AL::ALModule {
    public:
        KinematicsModule(boost::shared_ptr<AL::ALBroker> pBroker,
                         const std::string &pName);

        virtual ~KinematicsModule();

        virtual void init();

    private:
        void update();
        void apply();

        void setJoints(const AL::ALValue &values);

        AL::ALValue getJoints();

        AL::ALValue getCenterOfMass();

        void setHeadPosition(AL::ALValue pos, bool top_camera = true);
        void setLeftHandPosition(AL::ALValue pos);
        void setLeftLegPosition(AL::ALValue pos);
        void setRightHandPosition(AL::ALValue pos);
        void setRightLegPosition(AL::ALValue pos);

        AL::ALValue getHeadPosition(bool top_camera=true);
        AL::ALValue getLeftHandPosition();
        AL::ALValue getLeftLegPosition();
        AL::ALValue getRightHandPosition();
        AL::ALValue getRightLegPosition();

        void initFastAccess();
        void initKinematics();
        void initHW();

        void calculateJoints();

        inline AL::ALValue getPosition(NAOKinematics::Effectors ef);
        inline NAOKinematics::FKvars prepareFKvars(const AL::ALValue& pos);

        boost::mutex joints_mut;
        boost::mutex positions_mut;
        boost::mutex kinematics_mut;

        boost::shared_ptr<AL::ALProxy> hw;
        boost::shared_ptr<AL::ALMemoryFastAccess> mem;
        boost::shared_ptr<NAOKinematics> kinematics;
        std::vector<std::string> sensor_keys;
        std::vector<float> joint_values;
        std::vector<NAOKinematics::FKvars> positions;
        std::vector<bool> positions_mask;
        bool top_camera;
    };
}

#endif //NAOMECH_KINEMATICSMODULE_H
