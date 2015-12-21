//
// Created by arssivka on 11/27/15.
//

#ifndef NAOMECH_SENSORDATA_H
#define NAOMECH_SENSORDATA_H

#include <boost/shared_ptr.hpp>
#include <xmlrpc-c/registry.hpp>

namespace rd {
    template<typename T>
    class SensorData {
    public:
        SensorData() { }

        SensorData(const int length,
                   const int timestamp)
                : data(length), timestamp(timestamp) { }

        SensorData(const std::vector<T> &data,
                   const int timestamp)
                : data(data), timestamp(timestamp) { }

        std::vector<T> data;
        int timestamp;
    };
}


#endif //NAOMECH_SENSORDATA_H
