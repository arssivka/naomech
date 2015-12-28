//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_GYRO_H
#define NAOMECH_GYRO_H

#include <alcommon/albroker.h>
#include <rd/representation/SensorData.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>

namespace rd {
    class Gyro {
    public:
        enum Key {
            REF, // What this is kind of pokemon?!
            X,
            Y,
            SENSOR_COUNT
        };

        Gyro(boost::shared_ptr<AL::ALBroker> broker);

        const std::vector<std::string> &getKeys();

        boost::shared_ptr<SensorData<double> > getAngularVelocity();

    private:
        std::vector<std::string> keys;
        boost::shared_ptr<AL::ALMemoryProxy> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;

        AL::ALValue sensors;
    };
}

#endif //NAOMECH_GYRO_H
