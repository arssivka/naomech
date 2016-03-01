//
// Created by arssivka on 2/24/16.
//

#ifndef NAOMECH_LOCOMOTION_H
#define NAOMECH_LOCOMOTION_H


#include <boost/property_tree/ptree.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/atomic.hpp>
#include <nb/MetaGait.h>
#include <nb/StepGenerator.h>
#include "Joints.h"

namespace rd {
    class Locomotion {
    public:
        enum Parameters {
            X,
            Y,
            THETA,
            STEP_COUNT,
            PARAMETERS_COUNT
        };

        enum Odometry {
            X_OFFSET,
            Y_OFFSET,
            ROTATION,
            ODOMETRY_COUNT
        };

        Locomotion(boost::shared_ptr<Robot> robot);

        const StringKeyVector& getParameterKeys() const;

        const StringKeyVector& getOdometryKeys() const;
        
        const StringKeyVector& getJointKeys() const;

        SensorData<double>::Ptr getSpeedParameters(const StringKeyVector& keys);

        SensorData<double>::Ptr getSpeedParameters(const IntegerKeyVector& keys);

        void setSpeedParameters(const StringKeyVector& keys, const ValuesVector& values);

        void setSpeedParameters(const IntegerKeyVector& keys, const ValuesVector& values);

        rd::SensorData<double>::Ptr getOdometry();

        void resetOdometry();

        void setGaitParameters(const double stance_config[WP::LEN_STANCE_CONFIG],
                               const double step_config[WP::LEN_STEP_CONFIG],
                               const double zmp_config[WP::LEN_ZMP_CONFIG],
                               const double joint_hack_config[WP::LEN_HACK_CONFIG],
                               const double sensor_config[WP::LEN_SENSOR_CONFIG],
                               const double stiffness_config[WP::LEN_STIFF_CONFIG],
                               const double odo_config[WP::LEN_ODO_CONFIG],
                               const double arm_config[WP::LEN_ARM_CONFIG]);

        void generateStep();

        SensorData<double>::Ptr getPositions();

        SensorData<double>::Ptr getHardness();

        void applyPositions();

        bool isDone();

        SensorData<double>::Ptr getStanceJointData();

        void setAutoApplyUSleepTime(useconds_t us);

        useconds_t getAutoApplyUSleepTime() const;

        void setAutoApply(bool enable);

        bool isAutoApplyEnabled() const;

        virtual ~Locomotion();

    private:
        void autoUpdater();

    private:
        double m_x;
        double m_y;
        double m_theta;
        unsigned int m_step_count;

        boost::mutex m_access;

        SensorData<double>::Ptr m_stance_joints_data;
        SensorData<double>::Ptr m_hardness_data;
        SensorData<double>::Ptr m_positions_data;
        
        boost::scoped_ptr<MetaGait> m_meta_gait;
        boost::scoped_ptr<StepGenerator> m_step_generator;

        boost::shared_ptr<Joints> m_joints;
        boost::shared_ptr<Clock> m_clock;
        
        StringKeyVector m_joint_keys;
        StringKeyVector m_parameter_keys;
        StringKeyVector m_odometry_keys;

        std::map<std::string, int> m_parameters_key_map;

        boost::mutex m_auto_apply_mut;
        boost::atomic<bool> m_auto_apply_flag;
        boost::condition_variable m_auto_apply_cv;
        boost::thread m_auto_apply_worker;

        boost::atomic<bool> m_destroy;
        boost::atomic<useconds_t> m_auto_update_sleep_time;
    };
}


#endif //NAOMECH_LOCOMOTION_H
