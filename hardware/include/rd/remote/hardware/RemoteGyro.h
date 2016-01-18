//
// Created by arssivka on 12/28/15.
//

#ifndef NAOMECH_REMOTEGYRO_H
#define NAOMECH_REMOTEGYRO_H


#include <rd/hardware/Gyro.h>
#include <rd/network/RemoteModule.h>

namespace rd {
    class RemoteGyro : public RemoteModule {
    public:
        RemoteGyro(boost::shared_ptr<Gyro> gyro);

    private:
        class KeysMethod : public RemoteMethod {
        public:
            KeysMethod(boost::shared_ptr<Gyro> gyro);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            xmlrpc_c::value keys;
        };

        class AngularVelocityMethod : public RemoteMethod {
        public:
            AngularVelocityMethod(boost::shared_ptr<Gyro> gyro);

            virtual void execute(xmlrpc_c::paramList const &paramList,
                                 xmlrpc_c::value *const resultP);

        private:
            boost::shared_ptr<Gyro> gyro;
        };
    };
}


#endif //NAOMECH_REMOTEGYRO_H
