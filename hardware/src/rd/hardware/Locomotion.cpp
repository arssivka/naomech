//
// Created by arssivka on 2/24/16.
//

#include "rd/hardware/Locomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::Locomotion::Locomotion(boost::shared_ptr<Robot> robot)
        : m_meta_gait(new MetaGait), m_step_generator(new StepGenerator(robot, m_meta_gait.get())),
          m_joints(robot->getJoints()), m_clock(robot->getClock()), m_joint_keys(20), m_parameter_keys(PARAMETERS_COUNT),
          m_odometry_keys(ODOMETRY_COUNT) {
    Gait default_gait(DEFAULT_GAIT);
    m_meta_gait->setStartGait(default_gait);
    m_meta_gait->setNewGaitTarget(default_gait);

    m_stance_joints_data = boost::make_shared<SensorData<double> >(*m_step_generator->getDefaultStance(DEFAULT_GAIT),
                                                                   m_clock->getTime());

    m_parameter_keys[X] = "X";
    m_parameter_keys[Y] = "Y";
    m_parameter_keys[THETA] = "THETA";
    m_parameter_keys[STEP_COUNT] = "STEP_COUNT";

    for (int i = 0; i < PARAMETERS_COUNT; ++i) {
        m_parameters_key_map.insert(std::make_pair(m_parameter_keys[i], i));
    }

    m_odometry_keys[X_OFFSET] = "X_OFFSET";
    m_odometry_keys[Y_OFFSET] = "Y_OFFSET";
    m_odometry_keys[ROTATION] = "ROTATION";
    
    for (int i = 0; i < 20; ++i) {
        const std::vector<std::string>& keys = m_joints->getKeys();
        m_joint_keys[i] = keys[StepGenerator::NB_WALKING_JOINTS[i]];
    }

    int now = m_clock->getTime();
    m_positions_data = boost::make_shared<SensorData<double> >(m_joint_keys.size(), now);
    m_hardness_data = boost::make_shared<SensorData<double> >(m_joint_keys.size(), now);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::vector<std::string>& rd::Locomotion::getParameterKeys() const {
    return m_parameter_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getSpeedParameters(const std::vector<std::string>& keys) {
    std::vector<int> keys_i(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        std::map<std::string, int>::const_iterator found = m_parameters_key_map.find(keys[i]);
        if (found == m_parameters_key_map.end())
            return boost::make_shared<SensorData<double> >();
        keys_i[i] = found->second;
    }
    return this->getSpeedParameters(keys_i);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getSpeedParameters(const std::vector<int>& keys) {
    SensorData<double>::Ptr data = boost::make_shared<SensorData<double> >(keys.size(), m_clock->getTime());
    boost::lock_guard<boost::mutex> lock(m_access);
    for (int i = 0; i < keys.size(); ++i) {
        switch (keys[i]) {
            case X:
                data->data[i] = m_x;
                break;
            case Y:
                data->data[i] = m_y;
                break;
            case THETA:
                data->data[i] = m_theta;
                break;
            case STEP_COUNT:
                data->data[i] = m_step_count;
                break;
            default:
                return boost::make_shared<SensorData<double> >();
                break;
        }
    }
    return data;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setSpeedParameters(const std::vector<std::string>& keys, const std::vector<double>& values) {
    std::vector<int> keys_i(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        const std::map<std::string, int>::const_iterator found = m_parameters_key_map.find(keys[i]);
        if (found == m_parameters_key_map.end())
            return;
        keys_i[i] = found->second;
    }
    this->setSpeedParameters(keys_i, values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setSpeedParameters(const std::vector<int>& keys, const std::vector<double>& values) {
    if (keys.empty() || keys.size() != values.size())
        return;
    bool count_changed = false;
    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0.0;
    boost::lock_guard<boost::mutex> lock(m_access);
    for (int i = 0; i < keys.size(); ++i) {
        switch (keys[i]) {
            case X_OFFSET:
                m_x = values[i];
                break;
            case Y_OFFSET:
                m_y = values[i];
                break;
            case THETA:
                m_theta = values[i];
                break;
            case STEP_COUNT:
                count_changed = true;
                m_step_count = (unsigned int) values[i];
                break;
        }
    }
    if (count_changed) {
        m_step_generator->takeSteps(m_x, m_y, m_theta, m_step_count);
    } else {
        m_step_count = 0;
        m_step_generator->setSpeed(m_x, m_y, m_theta);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getOdometry() {
    boost::lock_guard<boost::mutex> lock(m_access);
    return boost::make_shared<SensorData<double> >(m_step_generator->getOdometryUpdate(), m_clock->getTime());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::resetOdometry() {
    boost::lock_guard<boost::mutex> lock(m_access);
    m_step_generator->resetSteps(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setGaitParameters(const double stance_config[WP::LEN_STANCE_CONFIG],
                                       const double step_config[WP::LEN_STEP_CONFIG],
                                       const double zmp_config[WP::LEN_ZMP_CONFIG],
                                       const double joint_hack_config[WP::LEN_HACK_CONFIG],
                                       const double sensor_config[WP::LEN_SENSOR_CONFIG],
                                       const double stiffness_config[WP::LEN_STIFF_CONFIG],
                                       const double odo_config[WP::LEN_ODO_CONFIG],
                                       const double arm_config[WP::LEN_ARM_CONFIG]) {
    boost::lock_guard<boost::mutex> lock(m_access);
    Gait next_gait(stance_config, step_config, zmp_config, joint_hack_config,
                         sensor_config, stiffness_config, odo_config, arm_config);
    m_meta_gait->setNewGaitTarget(next_gait);
    m_stance_joints_data = boost::make_shared<SensorData<double> >(*m_step_generator->getDefaultStance(next_gait),
                                                                   m_clock->getTime());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::vector<std::string>& rd::Locomotion::getJointKeys() const {
    return m_joint_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::generateStep() {
    int now = m_clock->getTime();
    boost::lock_guard<boost::mutex> lock(m_access);
    m_positions_data = boost::make_shared<SensorData<double> >(m_joint_keys.size(), now);
    m_hardness_data = boost::make_shared<SensorData<double> >(m_joint_keys.size(), now);
    m_step_generator->tick_controller();
    const WalkLegsTuple& legs = m_step_generator->tick_legs();
    const WalkArmsTuple& arms = m_step_generator->tick_arms();
    const std::vector<double>& lleg_joints = legs.get<LEFT_FOOT>().get<JOINT_INDEX>();
    const std::vector<double>& rleg_joints = legs.get<RIGHT_FOOT>().get<JOINT_INDEX>();
    const std::vector<double>& lleg_gains = legs.get<LEFT_FOOT>().get<STIFF_INDEX>();
    const std::vector<double>& rleg_gains = legs.get<RIGHT_FOOT>().get<STIFF_INDEX>();
    const std::vector<double>& larm_joints = arms.get<LEFT_FOOT>().get<JOINT_INDEX>();
    const std::vector<double>& rarm_joints = arms.get<RIGHT_FOOT>().get<JOINT_INDEX>();
    const std::vector<double>& larm_gains = arms.get<LEFT_FOOT>().get<STIFF_INDEX>();
    const std::vector<double>& rarm_gains = arms.get<RIGHT_FOOT>().get<STIFF_INDEX>();
    std::copy(larm_gains.begin(), larm_gains.end(), m_hardness_data->data.begin() + 0);
    std::copy(lleg_gains.begin(), lleg_gains.end(), m_hardness_data->data.begin() + 4);
    std::copy(rleg_gains.begin(), rleg_gains.end(), m_hardness_data->data.begin() + 10);
    std::copy(rarm_gains.begin(), rarm_gains.end(), m_hardness_data->data.begin() + 16);
    std::copy(larm_joints.begin(), larm_joints.end(), m_positions_data->data.begin() + 0);
    std::copy(lleg_joints.begin(), lleg_joints.end(), m_positions_data->data.begin() + 4);
    std::copy(rleg_joints.begin(), rleg_joints.end(), m_positions_data->data.begin() + 10);
    std::copy(rarm_joints.begin(), rarm_joints.end(), m_positions_data->data.begin() + 16);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getPositions() {
    boost::lock_guard<boost::mutex> lock(m_access);
    return m_positions_data;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getHardness() {
    boost::lock_guard<boost::mutex> lock(m_access);
    return m_hardness_data;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool rd::Locomotion::isDone() {
    boost::lock_guard<boost::mutex> lock(m_access);
    return m_step_generator->isDone();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const std::vector<std::string>& rd::Locomotion::getOdometryKeys() const {
    return m_odometry_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::applyPositions() {
    boost::lock_guard<boost::mutex> lock(m_access);
    m_joints->setHardness(m_joint_keys, m_hardness_data->data);
    m_joints->setPositions(m_joint_keys, m_positions_data->data);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getStanceJointData() {
    boost::lock_guard<boost::mutex> lock(m_access);
    return m_stance_joints_data;
}
