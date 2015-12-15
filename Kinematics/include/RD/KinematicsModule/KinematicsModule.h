//
// Created by arssivka on 10/5/15.
//

#ifndef NAOMECH_KINEMATICSMODULE_H
#define NAOMECH_KINEMATICSMODULE_H

#include <alcommon/almodule.h>
#include <boost/shared_ptr.hpp>
#include "Tools/Math/Pose3D.h"

#include "RD/HardwareAccessModule/HardwareDefines.h"

#include "Representations/Configuration/JointCalibration.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/MassCalibration.h"


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

        void lookAt(const AL::ALValue &pos,
                    const bool bot_camera = true);

        void setLeftArmPosition(const AL::ALValue &pos);

        void setLeftLegPosition(const AL::ALValue &pos);

        void setLeftLegPositionWithZ(const AL::ALValue &pos, float joint0);

        void setRightLegPositionWithZ(const AL::ALValue &pos, float joint0);

        void setRightArmPosition(const AL::ALValue &pos);

        void setRightLegPosition(const AL::ALValue &pos);

        AL::ALValue getHeadPosition(bool top_camera=true);

        AL::ALValue getLeftArmPosition();
        AL::ALValue getLeftLegPosition();

        AL::ALValue getRightArmPosition();
        AL::ALValue getRightLegPosition();

        void initFastAccess();
        void initHW();

        void calculateJoints();

        Pose3D preparePose3D(const AL::ALValue &pos);

        boost::mutex joints_mut;
        boost::mutex positions_mut;

        boost::shared_ptr<AL::ALProxy> hw;
        boost::shared_ptr<AL::ALMemoryFastAccess> mem;
        std::vector<std::string> sensor_keys;
        JointContainer joints;
        std::vector<Pose3D> effectors;
        std::vector<bool> effectors_mask;
        bool top_camera;
        float leftjoint0;
        Vector3<> obj;

        //TODO Delete this shit!

        JointCalibration jc;
        RobotDimensions rd;
        MassCalibration mc;
        CameraCalibration cc;
    };
}

#endif //NAOMECH_KINEMATICSMODULE_H
