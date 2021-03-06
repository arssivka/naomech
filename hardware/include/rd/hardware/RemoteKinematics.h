//
// Created by arssivka on 1/18/16.
//

#ifndef NAOMECH_REMOTEKINEMATICS_H
#define NAOMECH_REMOTEKINEMATICS_H


#include <rd/network/RemoteMethod.h>
#include <rd/network/RemoteModule.h>
#include <rd/hardware/Kinematics.h>
#include <xmlrpc-c/base.hpp>

namespace rd {
    class RemoteKinematics : public RemoteModule {
    public:
        RemoteKinematics(boost::shared_ptr<Kinematics> kinematics);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            KeysMethod(boost::shared_ptr<Kinematics> kinematics);

            virtual void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP);

        private:
            xmlrpc_c::value keys;
        };

        class LookAtMethod : public RemoteMethod {
        public:
            LookAtMethod(boost::shared_ptr<Kinematics> kinematics);

            virtual void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP);

        private:
            boost::shared_ptr<Kinematics> m_kinematics;
        };

        class JointsLookAtMethodLookAtMethod : public RemoteMethod {
        public:
            JointsLookAtMethodLookAtMethod(boost::shared_ptr<Kinematics> kinematics);

            virtual void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP);

        private:
            boost::shared_ptr<Kinematics> m_kinematics;
        };

        class GetHeadMethod : public RemoteMethod {
        public:
            GetHeadMethod(boost::shared_ptr<Kinematics> kinematics);

            virtual void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP);

        private:
            boost::shared_ptr<Kinematics> m_kinematics;
        };

        class PositionMethod : public RemoteMethod {
        public:
            PositionMethod(boost::shared_ptr<Kinematics> kinematics);

            virtual void execute(xmlrpc_c::paramList const& paramList, xmlrpc_c::value* const resultP);

        private:
            boost::shared_ptr<Kinematics> m_kinematics;
        };
    };
}


#endif //NAOMECH_REMOTEKINEMATICS_H
