//
// Created by arssivka on 2/29/16.
//

#ifndef NAOMECH_REMOTELOCOMOTION_H
#define NAOMECH_REMOTELOCOMOTION_H


#include <rd/network/RemoteMethod.h>
#include <rd/network/RemoteModule.h>
#include "rd/hardware/Locomotion.h"

namespace rd {
    class RemoteLocomotion : public RemoteModule {
    public:

        RemoteLocomotion(boost::shared_ptr<Locomotion> locomotion);

    private:
        class ParameterKeysMethod : public RemoteMethod {
        public:
            ParameterKeysMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value m_keys;
        };

        class OdometryKeysMethod : public RemoteMethod {
        public:
            OdometryKeysMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value m_keys;
        };

        class JointKeysMethod : public RemoteMethod {
        public:
            JointKeysMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList, xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value m_keys;
        };

        class ParametersMethod : public RemoteMethod {
        public:
            ParametersMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };

        class OdometryMethod : public RemoteMethod {
        public:
            OdometryMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };

        class GaitParametersMethod : public RemoteMethod {
        public:
            GaitParametersMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };

        class JointsPositionsMethod : public RemoteMethod {
        public:
            JointsPositionsMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };

        class JointsHardnessMethod : public RemoteMethod {
        public:
            JointsHardnessMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };

        class JointsApplyMethod : public RemoteMethod {
        public:
            JointsApplyMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };

        class IsDoneMethod : public RemoteMethod {
        public:
            IsDoneMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };

        class GenerateStepMethod : public RemoteMethod {
        public:
            GenerateStepMethod(boost::shared_ptr<Locomotion> locomotion);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Locomotion> m_locomotion;
        };
    };
};


#endif //NAOMECH_REMOTELOCOMOTION_H
