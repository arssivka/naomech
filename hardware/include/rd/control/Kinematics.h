//
// Created by arssivka on 1/18/16.
//

#ifndef NAOMECH_KINEMATICS_H
#define NAOMECH_KINEMATICS_H


#include <rd/hardware/Joints.h>
#include <boost/property_tree/ptree.hpp>
#include <bhuman/representations/configuration/JointCalibration.h>
#include <bhuman/representations/configuration/RobotDimensions.h>
#include <bhuman/representations/configuration/MassCalibration.h>
#include <bhuman/representations/configuration/CameraCalibration.h>

namespace rd {
    class Kinematics : boost::noncopyable {
    public:
        enum Key {
            LEFT_ARM_X,
            LEFT_ARM_Y,
            LEFT_ARM_Z,
            RIGHT_ARM_X,
            RIGHT_ARM_Y,
            RIGHT_ARM_Z,
            LEFT_LEG_X,
            LEFT_LEG_Y,
            LEFT_LEG_Z,
            LEFT_LEG_ROLL,
            LEFT_LEG_PITCH,
            LEFT_LEG_YAW,
            RIGHT_LEG_X,
            RIGHT_LEG_Y,
            RIGHT_LEG_Z,
            RIGHT_LEG_ROLL,
            RIGHT_LEG_PITCH,
            RIGHT_LEG_YAW,
            KEYS_COUNT
        };

        Kinematics(boost::shared_ptr<Clock> clock, boost::shared_ptr<Joints> joints,
                   boost::shared_ptr<boost::property_tree::ptree> config);

        const std::vector<std::string>& getKeys() const;

        void lookAt(double x, double y, double z, bool top_camera);

        void setPosition(const std::vector<std::string>& keys, const std::vector<double>& values);

        void setPosition(const std::vector<int>& keys, const std::vector<double>& values);

        boost::shared_ptr<SensorData<double> > getPosition(const std::vector<std::string>& keys);

        boost::shared_ptr<SensorData<double> > getPosition(const std::vector<int>& keys);

        boost::shared_ptr<SensorData<double> > getPosition();

    private:
        enum Limb {
            LEFT_ARM,
            RIGHT_ARM,
            LEFT_LEG,
            RIGHT_LEG,
            LIMBS_COUNT
        };

        enum LimbMask {
            LEFT_ARM_MASK = 1 << LEFT_ARM_X |
                            1 << LEFT_ARM_Y |
                            1 << LEFT_ARM_Z,
            RIGHT_ARM_MASK = 1 << RIGHT_ARM_X |
                             1 << RIGHT_ARM_Y |
                             1 << RIGHT_ARM_Z,
            LEFT_LEG_MASK = 1 << LEFT_LEG_X |
                            1 << LEFT_LEG_Y |
                            1 << LEFT_LEG_Z |
                            1 << LEFT_LEG_ROLL |
                            1 << LEFT_LEG_PITCH |
                            1 << LEFT_LEG_YAW,
            RIGHT_LEG_MASK = 1 << RIGHT_LEG_X |
                             1 << RIGHT_LEG_Y |
                             1 << RIGHT_LEG_Z |
                             1 << RIGHT_LEG_ROLL |
                             1 << RIGHT_LEG_PITCH |
                             1 << RIGHT_LEG_YAW,
            MASKS_COUNT = 4
        };

        std::vector<std::string> keys;
        std::map<std::string, int> keys_map;

        boost::shared_ptr<Clock> clock;
        boost::shared_ptr<Joints> joints;

        JointCalibration joint_calibration;
        RobotDimensions robot_dimensions;
        MassCalibration mass_calibration;
        CameraCalibration camera_calibration;
    };
}


#endif //NAOMECH_KINEMATICS_H
