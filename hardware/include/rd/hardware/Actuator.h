//
// Created by arssivka on 11/27/15.
//

#ifndef NAOMECH_ACTUATOR_H
#define NAOMECH_ACTUATOR_H


#include <rd/hardware/Sensor.h>


namespace rd {
    template<typename T>
    class Actuator : public Sensor<T> {
    public:
        Actuator(const std::string &name,
                 const boost::shared_ptr<AL::ALMemoryProxy> &mem,
                 const boost::shared_ptr<AL::DCMProxy> &dcm)
                : Sensor<T>(name, mem, dcm) { }

        virtual const std::vector<std::string> &getInputKeys() const = 0;

        virtual bool set(const std::vector<std::string> &keys,
                         const std::vector<T> &values, int time_offset) = 0;

        virtual bool set(const std::vector<int> &keys,
                         const std::vector<T> &values, int time_offset) = 0;

    protected:
        template<typename T1, typename T2>
        bool setValues(const T1 &keys_container,
                       const std::vector<T2> &keys,
                       const std::vector<T> &values, int time_offset) {
            int length = keys.size();
            if (length != values.size()) {
                return false;
            }

            std::vector<std::string> dcm_keys(length);
            for (int i = 0; i < length; ++i) {
                try {
                    dcm_keys[i] = keys_container.at(keys.at(i));
                } catch (...) {
                    return false;
                }
            }

            // Update alias
            AL::ALValue cmd;
            cmd.arraySetSize(2);
            cmd[0] = this->name;
            cmd[1].arraySetSize(length);
            for (int i = 0; i < length; ++i) {
                cmd[1][i] = keys[i];
            }
            this->dcm->createAlias(cmd);
            // Send command
            cmd.arraySetSize(1);
            cmd[0] = this->name;
            cmd[1] = std::string("ClearAll");
            cmd[2] = std::string("time-separate");
            cmd[3] = 0;
            cmd[4].arraySetSize(1);
            cmd[4][0] = this->clock(time_offset);
            cmd[5].arraySetSize(length);
            for (int i = 0; i < length; ++i) {
                cmd[5][i].arraySetSize(1);
                cmd[5][i][0] = values[i];
            }
            this->dcm->setAlias(cmd);
            return true;
        }
    };
}


#endif //NAOMECH_ACTUATOR_H
