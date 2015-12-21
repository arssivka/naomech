//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_MOTOR_H
#define NAOMECH_MOTOR_H


#include <vector>
#include <map>
#include <alproxies/dcmproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alcommon/alproxy.h>

#include <rd/representation/SensorData.h>
#include <rd/hardware/Clock.h>


namespace rd {
    class Joints {
    public:
        enum Key {
            HEAD_YAW, HEAD_PITCH,
            L_SHOULDER_PITCH,
            L_SHOULDER_ROLL,
            L_ELBOW_YAW,
            L_ELBOW_ROLL,
            L_WRIST_YAW,
            L_HAND,
            L_HIP_YAW_PITCH,
            L_HIP_ROLL,
            L_HIP_PITCH,
            L_KNEE_PITCH,
            L_ANKLE_PITCH,
            L_ANKLE_ROLL,
            R_HIP_YAW_PITCH,
            R_HIP_ROLL,
            R_HIP_PITCH,
            R_KNEE_PITCH,
            R_ANKLE_PITCH,
            R_ANKLE_ROLL,
            R_SHOULDER_PITCH,
            R_SHOULDER_ROLL,
            R_ELBOW_YAW,
            R_ELBOW_ROLL,
            R_WRIST_YAW,
            R_HAND,
            JOINTS_COUNT
        };


        Joints(boost::shared_ptr<AL::ALBroker> brocker);

        const std::vector<std::string> &getKeys() const;

        bool setPosition(const std::vector<std::string> &keys,
                         const std::vector<double> &values);

        bool setPosition(const std::vector<int> &keys,
                         const std::vector<double> &values);

        boost::shared_ptr<SensorData<double> > getPosition(const std::vector<int> &keys);

        boost::shared_ptr<SensorData<double> > getPosition(const std::vector<std::string> &keys);

        boost::shared_ptr<SensorData<double> > getPosition();

        bool setHardness(double value);

        bool setHardness(const std::vector<std::string> &keys,
                         const std::vector<double> &values);

        bool setHardness(const std::vector<int> &keys,
                         const std::vector<double> &values);

        boost::shared_ptr<SensorData<double> > getHardness(const std::vector<int> &keys);

        boost::shared_ptr<SensorData<double> > getHardness(const std::vector<std::string> &keys);

        boost::shared_ptr<SensorData<double> > getHardness();

    private:
        const static std::string DCM_POSITION_ALIAS;
        const static std::string DCM_HARDNESS_ALIAS;

        boost::shared_ptr<AL::ALMemoryProxy> mem;
        boost::shared_ptr<AL::DCMProxy> dcm;
        boost::shared_ptr<AL::ALProxy> hw;

        boost::mutex synch;
        AL::ALValue dcm_cmd;

        std::vector<std::string> keys;

        AL::ALValue position_in_list;
        AL::ALValue position_out_list;
        AL::ALValue hardness_list;
        std::map<std::string, int> out_map;
        std::map<std::string, std::string> position_map;
        std::map<std::string, std::string> hardness_map;

        void makeAlias(const std::string &name, const AL::ALValue &keys);

        void initKeysMap(std::map<std::string, std::string> &container,
                         const std::vector<std::string> &keys,
                         const std::vector<std::string> &values);
    };
}


#endif //NAOMECH_MOTOR_H
