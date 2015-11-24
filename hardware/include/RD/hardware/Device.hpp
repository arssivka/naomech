#ifndef NAOMECH_DEVICE_H
#define NAOMECH_DEVICE_H

#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <alproxies/almemoryproxy.h>
#include <alproxies/dcmproxy.h>
#include "Clock.h"


namespace RD {
    template<typename T>
    class SensorData {
    public:
        SensorData(const boost::shared_ptr<std::vector<T> > &data,
                   const int timestamp)
                : data(data), timestamp(timestamp) { }

        boost::shared_ptr<std::vector<T> > data;
        int timestamp;
    };


    template<typename T>
    class Sensor {
    public:
        Sensor(const std::string &name,
               boost::shared_ptr<AL::ALMemoryProxy> memory,
               const boost::shared_ptr<AL::DCMProxy> dcm,
               std::vector<std::string> keys)
                : clock(dcm), dcm(dcm), mem(memory), name(name), keys(keys) { }

        const std::string &getName() const {
            return this->name;
        }

        const std::vector<std::string> &getKeyNames() const {
            return this->keys;
        }

        virtual SensorData<T> get() {
            unsigned int length = this->sensor_keys.getSize();
            int timestamp = this->clock.getTime();
            AL::ALValue val = this->mem->getListData(this->sensor_keys);
            boost::shared_ptr<std::vector<T> > data =
                    boost::make_shared<std::vector<T> >(length);
            for (unsigned int i = 0; i < length; ++i) {
                data->at(i) = val[i];
            }
            return SensorData(data, timestamp);
        }

        virtual ~Sensor() { }

    protected:
        bool updateSensorKeys(std::vector<std::string> mem_keys) {
            unsigned int length = mem_keys.size();
            if (length != this->keys.size()) {
                return false;
            }
            for (int i = 0; i < length; ++i) {
                this->sensor_keys[i] = mem_keys[i];
            }
            return true;
        }

        const Clock clock;
        boost::shared_ptr<AL::DCMProxy> dcm;
        boost::shared_ptr<AL::ALMemoryProxy> mem;

    private:
        const std::string name;
        std::vector<std::string> keys;
        AL::ALValue sensor_keys;
    };


    template<typename T>
    class Actuator : public Sensor<T> {
    public:
        Actuator(const std::string &name,
                 const boost::shared_ptr<AL::ALMemoryProxy> mem,
                 std::vector<std::string> keys,
                 const boost::shared_ptr<AL::DCMProxy> dcm)
                : Sensor(name, mem, dcm, keys) { }

        bool set(const std::vector<std::string> &keys,
                 const std::vector<T> &values, int time) {
            int length = keys.size();
            if (length != values.size()) {
                return false;
            }
            std::vector<std::string> dcm_keys(length);
            for (int i = 0; i < length; ++i) {
                dcm_keys[i] = this->dcm_key_map[keys[i]];
            }
            this->setValues(dcm_keys, values, time);
            return true;
        }

        bool set(const std::vector<unsigned int> &keys,
                 const std::vector<T> &values, int time) {
            int length = keys.size();
            if (length != values.size()) {
                return false;
            }
            std::vector<std::string> dcm_keys(length);
            for (int i = 0; i < length; ++i) {
                dcm_keys[i] = this->actuator_keys[keys[i]];
            }
            this->setValues(dcm_keys, values, time);
            return true;
        }

    protected:
        bool updateActuatorKeys(std::vector<std::string> actuator_keys) {
            const std::vector<std::string> &keys = this->getKeyNames();
            unsigned int length = actuator_keys.size();
            if (length != keys.size()) {
                return false;
            }
            for (int i = 0; i < length; ++i) {
                this->dcm_key_map.insert(std::make_pair(keys[i],
                                                        actuator_keys[i]));
            }
            this->actuator_keys = actuator_keys;
            return true;
        }

        void setValues(const std::vector<std::string> &keys,
                       const std::vector<T> &values, int time) {
            int length = keys.size();
            // Update alias
            AL::ALValue cmd;
            cmd.arraySetSize(2);
            cmd[0] = this->getName();
            cmd[1].arraySetSize(length);
            for (int i = 0; i < length; ++i) {
                cmd[1][i] = keys[i];
            }
            this->dcm->createAlias(cmd);
            // Send command
            // TODO Prepare command early
            cmd.arraySetSize(1);
            cmd[0] = this->getName();
            cmd[1] = std::string("ClearAll");
            cmd[2] = std::string("time-separate");
            cmd[3] = 0;
            cmd[4].arraySetSize(1);
            cmd[4][0] = time;
            cmd[5].arraySetSize(length);
            for (int i = 0; i < length; ++i) {
                cmd[5][i].arraySetSize(1);
                cmd[5][i][0] = values[i];
            }
            this->dcm->setAlias(cmd);
        }

    private:
        std::vector<std::string> actuator_keys;
        std::map<std::string, std::string> dcm_key_map;
    };
}


#endif //NAOMECH_DEVICE_H
