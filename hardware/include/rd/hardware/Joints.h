//
// Created by arssivka on 11/23/15.
//

#ifndef NAOMECH_MOTOR_H
#define NAOMECH_MOTOR_H


#include <rd/hardware/Actuator.h>


namespace rd {
    class Joints : public Actuator<float> {
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


        Joints(boost::shared_ptr<AL::ALMemoryProxy> memory,
               const boost::shared_ptr<AL::DCMProxy> dcm);

        virtual const std::vector<std::string> &getInputKeys() const;

        virtual bool set(const std::vector<std::string> &keys,
                         const std::vector<float> &values, int time_offset);

        virtual bool set(const std::vector<int> &keys,
                         const std::vector<float> &values, int time_offset);

        virtual const std::vector<std::string> &getOutputKeys() const;

        virtual SensorData<float> get(const std::vector<int> &keys);

        virtual SensorData<float> get(const std::vector<std::string> &keys);

        virtual SensorData<float> get();

        bool setHardness(float value, int time_offset);

        bool setHardness(const std::vector<std::string> &keys,
                         const std::vector<float> &values, int time_offset);

        bool setHardness(const std::vector<int> &keys,
                         const std::vector<float> &values, int time_offset);

        SensorData<float> getHardness(const std::vector<int> &keys);

        SensorData<float> getHardness(const std::vector<std::string> &keys);

        SensorData<float> getHardness();

    private:
        static void initKeysMap(std::map<std::string, std::string> &map,
                                const std::vector<std::string> &keys,
                                const std::vector<std::string> &values);

        std::vector<std::string> keys;

        std::vector<std::string> position_in_list;
        std::vector<std::string> position_out_list;
        std::map<std::string, std::string> position_in_map;
        std::map<std::string, std::string> position_out_map;

        std::vector<std::string> hardness_list;
        std::map<std::string, std::string> hardness_map;
    };
}


#endif //NAOMECH_MOTOR_H
