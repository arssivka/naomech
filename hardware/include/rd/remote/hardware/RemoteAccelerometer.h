//
// Created by arssivka on 1/18/16.
//

#ifndef NAOMECH_REMOTEACCELEROMETER_H
#define NAOMECH_REMOTEACCELEROMETER_H


#include <rd/hardware/Accelerometer.h>
#include <rd/network/RemoteModule.h>

namespace rd {
    class RemoteAccelerometer : public RemoteModule {
    public:
        RemoteAccelerometer(boost::shared_ptr<Accelerometer> gyro);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            KeysMethod(boost::shared_ptr<Accelerometer> gyro);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value keys;
        };

        class AngularVelocityMethod : public RemoteMethod {
        public:
            AngularVelocityMethod(boost::shared_ptr<Accelerometer> gyro);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Accelerometer> gyro;
        };
    };
}


#endif //NAOMECH_REMOTEACCELEROMETER_H
