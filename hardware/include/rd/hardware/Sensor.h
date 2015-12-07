//
// Created by arssivka on 11/27/15.
//

#ifndef NAOMECH_SENSOR_H
#define NAOMECH_SENSOR_H


#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include <rd/hardware/Clock.h>
#include <rd/hardware/SensorData.h>


namespace rd {
    template<typename T>
    class Sensor {
    public:
        Sensor(const std::string &name,
               const boost::shared_ptr<AL::ALMemoryProxy> &memory,
               const boost::shared_ptr<AL::DCMProxy> &dcm)
                : name(name), dcm(dcm), mem(memory), clock(dcm) { }

        const std::string &getName() const {
            return this->name;
        }

        virtual const std::vector<std::string> &getOutputKeys() const = 0;

        virtual SensorData<T> get(const std::vector<int> &keys) = 0;

        virtual SensorData<T> get(const std::vector<std::string> &keys) = 0;

        virtual SensorData<T> get() = 0;

        virtual ~Sensor() { }

    protected:
        template<typename T1, typename T2>
        boost::shared_ptr<std::vector<T> > getValues(const T1 &keys_container,
                                                     const std::vector<T2> &keys) {
            unsigned int length = keys.size();
            int timestamp = this->clock.getDCMTime();
            AL::ALValue data;
            data.arraySetSize(length);
            for (int i = 0; i < length; ++i) {
                try {
                    data[i] = keys_container.at(keys.at(i));
                } catch (...) {
                    return boost::make_shared<std::vector<float> >();
                }
            }

            data = this->mem->getListData(data);
            boost::shared_ptr<std::vector<T> > res =
                    boost::make_shared<std::vector<T> >(length);
            for (unsigned int i = 0; i < length; ++i) {
                res->at(i) = data[i];
            }
            return res;
        };

        const std::string name;
        boost::shared_ptr<AL::DCMProxy> dcm;
        boost::shared_ptr<AL::ALMemoryProxy> mem;
        const Clock clock;

    };
}


#endif //NAOMECH_SENSOR_H
