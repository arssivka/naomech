//
// Created by arssivka on 10/1/15.
//

#ifndef NAOKINEMATICS_DCMKINEMATICSMODULE_H
#define NAOKINEMATICS_DCMKINEMATICSMODULE_H

#include <vector>

#include <alerror/alerror.h>
#include <alcommon/almodule.h>
#include <almemoryfastaccess/almemoryfastaccess.h>
#include <alproxies/dcmproxy.h>
#include "libNAOKinematics/NAOKinematics.h"

namespace RD {
    class DCMKinematicsModule : public AL::ALModule {
    public:
        enum DCMUpdateType {
            ClearAll,
            Merge,
            ClearAfter,
            ClearBefore
        };

        typedef KDeviceLists::ChainsNames Chain;

        DCMKinematicsModule(boost::shared_ptr <AL::ALBroker> broker,
                            const std::string &name, DCMUpdateType utype = ClearAll);

        virtual ~DCMKinematicsModule();

        void setDCMUpdateType(DCMUpdateType utype);
        DCMUpdateType getDCMUpdateType() const;

        void setHeadPosition(const AL::ALValue &pos, int time=0);
        void setLHandPosition(const AL::ALValue &pos, int time=0);
        void setRHandPosition(const AL::ALValue &pos, int time=0);
        void setLLegPosition(const AL::ALValue &pos, int time=0);
        void setRLegPosition(const AL::ALValue &pos, int time=0);

    private:
        virtual void init();

        void startLoop();
        void stopLoop();

        void DCMCallback();

        DCMUpdateType utype;

        boost::shared_ptr<AL::DCMProxy> dcm;
        boost::shared_ptr<AL::ALMemoryFastAccess> mem;

        ProcessSignalConnection dcm_post_process_connection;

        std::vector<std::string> sensor_keys;
        std::vector<float> sensor_values;
        std::vector<float> initial_sensor_values;

        AL::ALValue commands;
    };
}


#endif //NAOKINEMATICS_DCMKINEMATICSMODULE_H
