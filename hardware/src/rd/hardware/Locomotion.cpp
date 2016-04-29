//
// Created by arssivka on 2/24/16.
//

#include <rd/hardware/TypeDefinition.h>
#include "rd/hardware/Locomotion.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::Locomotion::Locomotion(boost::shared_ptr<Robot> robot)
        : m_x(0.0), m_y(0.0), m_theta(0.0), m_step_count(0), m_meta_gait(new MetaGait),
          m_step_generator(new StepGenerator(robot, m_meta_gait.get())), m_joints(robot->getJoints()),
          m_clock(robot->getClock()), m_joint_keys(20), m_head_joint_keys(2), m_parameter_keys(PARAMETERS_COUNT),
          m_odometry_keys(ODOMETRY_COUNT), m_auto_apply_flag(false),
          m_auto_apply_worker(boost::bind(&rd::Locomotion::autoUpdater, this)), m_auto_update_sleep_time(1024) {
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
        m_parameter_keys_map.insert(std::make_pair(m_parameter_keys[i], i));
    }

    m_odometry_keys[X_OFFSET] = "X_OFFSET";
    m_odometry_keys[Y_OFFSET] = "Y_OFFSET";
    m_odometry_keys[ROTATION] = "ROTATION";

    const StringKeyVector& keys = m_joints->getKeys();
    for (int i = 0; i < 20; ++i) m_joint_keys[i] = keys[StepGenerator::NB_WALKING_JOINTS[i]];
    m_joint_keys.push_back(keys[rd::Joints::HEAD_PITCH]);
    m_joint_keys.push_back(keys[rd::Joints::HEAD_YAW]);

    int now = m_clock->getTime();
    m_positions_data = boost::make_shared<SensorData<double> >(ValuesVector(m_joint_keys.size(), 0.0), now);
    m_hardness_data = boost::make_shared<SensorData<double> >(ValuesVector(m_joint_keys.size(), 0.0), now);
    m_step_generator->setSpeed(m_x, m_y, m_theta);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const StringKeyVector& rd::Locomotion::getParameterKeys() const {
    return m_parameter_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getSpeedParameters(const StringKeyVector& keys) const {
    IntegerKeyVector keys_i(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        std::map<std::string, int>::const_iterator found = m_parameter_keys_map.find(keys[i]);
        if (found == m_parameter_keys_map.end())
            return boost::make_shared<SensorData<double> >();
        keys_i[i] = found->second;
    }
    return this->getSpeedParameters(keys_i);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getSpeedParameters(const IntegerKeyVector& keys) const {
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

void rd::Locomotion::setSpeedParameters(const StringKeyVector& keys, const ValuesVector& values) {
    IntegerKeyVector keys_i(keys.size());
    for (int i = 0; i < keys.size(); ++i) {
        const std::map<std::string, int>::const_iterator found = m_parameter_keys_map.find(keys[i]);
        if (found == m_parameter_keys_map.end())
            return;
        keys_i[i] = found->second;
    }
    this->setSpeedParameters(keys_i, values);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setSpeedParameters(const IntegerKeyVector& keys, const ValuesVector& values) {
    if (keys.empty() || keys.size() != values.size())
        return;
    boost::lock_guard<boost::mutex> lock(m_access);
    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0.0;
    m_step_count = 0;
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
                m_step_count = (unsigned int) values[i];
                break;
            default:
                continue;
        }
    }
    if (m_step_count > 0) {
        m_step_generator->takeSteps(m_x, m_y, m_theta, m_step_count);
    } else {
        m_step_generator->setSpeed(m_x, m_y, m_theta);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getOdometry() {
    boost::lock_guard<boost::mutex> lock(m_access);
    rd::SensorData<double>::Ptr data = boost::make_shared<SensorData<double> >(m_step_generator->getOdometryUpdate(), m_clock->getTime());
    data->data[1] *= -1.0; // hack!!!!!
    return data;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::resetOdometry() {
    boost::lock_guard<boost::mutex> lock(m_access);
    m_step_generator->resetOdometry(0.0, 0.0);
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

const StringKeyVector& rd::Locomotion::getJointKeys() const {
    return m_joint_keys;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::generateStep() {
    try {
        int now = m_clock->getTime();
        boost::lock_guard<boost::mutex> lock(m_access);
        if (!m_step_generator->isDone()) {
            double pitch_p = m_positions_data->data[20];
            double yaw_p = m_positions_data->data[21];
            double pitch_h = m_hardness_data->data[20];
            double yaw_h = m_hardness_data->data[21];
            m_positions_data = boost::make_shared<SensorData<double> >(m_joint_keys.size(), now);
            m_hardness_data = boost::make_shared<SensorData<double> >(m_joint_keys.size(), now);
            m_meta_gait->tick_gait();
            m_step_generator->tick_controller();
            const WalkLegsTuple& legs = m_step_generator->tick_legs();
            const WalkArmsTuple& arms = m_step_generator->tick_arms();
            const ValuesVector& lleg_joints = legs.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const ValuesVector& rleg_joints = legs.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const ValuesVector& lleg_gains = legs.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const ValuesVector& rleg_gains = legs.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            const ValuesVector& larm_joints = arms.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const ValuesVector& rarm_joints = arms.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const ValuesVector& larm_gains = arms.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const ValuesVector& rarm_gains = arms.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            std::copy(larm_gains.begin(), larm_gains.end(), m_hardness_data->data.begin() + 0);
            std::copy(lleg_gains.begin(), lleg_gains.end(), m_hardness_data->data.begin() + 4);
            std::copy(rleg_gains.begin(), rleg_gains.end(), m_hardness_data->data.begin() + 10);
            std::copy(rarm_gains.begin(), rarm_gains.end(), m_hardness_data->data.begin() + 16);
            m_hardness_data->data[20] = pitch_h;
            m_hardness_data->data[21] = yaw_h;
            std::copy(larm_joints.begin(), larm_joints.end(), m_positions_data->data.begin() + 0);
            std::copy(lleg_joints.begin(), lleg_joints.end(), m_positions_data->data.begin() + 4);
            std::copy(rleg_joints.begin(), rleg_joints.end(), m_positions_data->data.begin() + 10);
            std::copy(rarm_joints.begin(), rarm_joints.end(), m_positions_data->data.begin() + 16);
            m_positions_data->data[20] = pitch_p;
            m_positions_data->data[21] = yaw_p;
        } else {
            m_positions_data->timestamp = now;
            m_hardness_data->timestamp = now;
        }
    } catch (const char * e) {
        std::cerr << e << std::endl;
    }
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

const StringKeyVector& rd::Locomotion::getOdometryKeys() const {
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setAutoApply(bool enable) {
    if (enable) {
        boost::lock_guard<boost::mutex> lock(m_auto_apply_mut);
        m_auto_apply_flag.store(true);
        m_auto_apply_cv.notify_one();
    } else {
        boost::lock_guard<boost::mutex> lock(m_auto_apply_mut);
        m_auto_apply_flag.store(false);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool rd::Locomotion::isAutoApplyEnabled() const {
    return m_auto_apply_flag.load();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::Locomotion::~Locomotion() {
    boost::lock_guard<boost::mutex> lock(m_access);
    m_destroy.store(true);
    m_auto_apply_flag.store(true);
    m_auto_apply_cv.notify_one();
    m_auto_apply_worker.join();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::autoUpdater() {
    while (!m_destroy.load()) {
        boost::unique_lock<boost::mutex> lock(m_auto_apply_mut);
        while (!m_auto_apply_flag.load()) m_auto_apply_cv.wait(lock);
        this->generateStep();
        this->applyPositions();
        usleep(m_auto_update_sleep_time.load());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setAutoApplyUSleepTime(useconds_t us) {
    m_auto_update_sleep_time.store(us);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

useconds_t rd::Locomotion::getAutoApplyUSleepTime() const {
    return m_auto_update_sleep_time.load();
}

void rd::Locomotion::reset(bool left) {
    m_step_generator->setSpeed(0.0, 0.0, 0.0);
    m_step_generator->resetSteps(left);
    m_step_generator->resetHard();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setHeadPositions(double pitch, double yaw) {
    boost::lock_guard<boost::mutex> lock(m_access);
    m_positions_data->data[20] = pitch;
    m_positions_data->data[21] = yaw;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void rd::Locomotion::setHeadHardness(double pitch, double yaw) {
    boost::lock_guard<boost::mutex> lock(m_access);
    m_hardness_data->data[20] = pitch;
    m_hardness_data->data[21] = yaw;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getHeadPositions() const {
    boost::lock_guard<boost::mutex> lock(m_access);
    return m_positions_data;//boost::make_shared<SensorData<double> >(m_head_positions, m_clock->getTime());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

rd::SensorData<double>::Ptr rd::Locomotion::getHeadHardness() const {
    boost::lock_guard<boost::mutex> lock(m_access);
    return m_hardness_data;//boost::make_shared<SensorData<double> >(m_head_hardness, m_clock->getTime());
}
