//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_ACCELEROMETER_H
#define NAOMECH_ACCELEROMETER_H

#include <alcommon/albroker.h>
#include <rd/representation/SensorData.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>

namespace rd {
    class Accelerometer : boost::noncopyable {
    public:
        enum Key {
            X, // What this is kind of pokemon?!
            Y,
            Z,
            SENSOR_COUNT
        };

        Accelerometer(boost::shared_ptr<AL::ALBroker> broker);

        const std::vector<std::string> &getKeys();

        boost::shared_ptr<SensorData<double> > getAcceleration();

    private:
        std::vector<std::string> keys;
        boost::shared_ptr<AL::ALMemoryProxy> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;

        AL::ALValue sensors;
    };
}

#endif //NAOMECH_ACCELEROMETER_H
