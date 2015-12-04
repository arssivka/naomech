//
// Created by arssivka on 11/27/15.
//

#ifndef NAOMECH_REMOTEJOINTS_H
#define NAOMECH_REMOTEJOINTS_H

#include <rd/network/RemoteModule.h>
#include <rd/hardware/Joints.h>

namespace rd {
    class RemoteJoints : public RemoteModule {
    public:
        RemoteJoints(boost::shared_ptr<Joints> joints);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            KeysMethod(boost::shared_ptr<Joints> joints);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value keys;
        };

        class PositionMethod : public RemoteMethod {
        public:
            PositionMethod(boost::shared_ptr<Joints> joints);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Joints> joints;
        };

        class HardnessMethod : public RemoteMethod {
        public:
            HardnessMethod(boost::shared_ptr<Joints> joints);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Joints> joints;
        };
    };
}


#endif //NAOMECH_REMOTEJOINTS_H
