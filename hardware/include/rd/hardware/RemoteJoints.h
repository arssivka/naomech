//
// Created by arssivka on 11/27/15.
//

#ifndef NAOMECH_REMOTEJOINTS_H
#define NAOMECH_REMOTEJOINTS_H

#include <rd/network/RPCModule.h>
#include <rd/hardware/Joints.h>

namespace rd {
    class RemoteJoints : public RPCModule {
    public:
        RemoteJoints(boost::shared_ptr<Joints> joints);

    private:
        boost::shared_ptr<Joints> joints;
    };
}


#endif //NAOMECH_REMOTEJOINTS_H
