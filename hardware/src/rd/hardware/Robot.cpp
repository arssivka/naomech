//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Robot.h>
#include <alcommon/albrokermanager.h>
#include <boost/property_tree/json_parser.hpp>

using namespace AL;
using namespace boost;
using namespace std;
using namespace rd;

Robot::Robot(const std::string& name, const std::string& ip, unsigned int port, const std::string& config_filename)
        : m_config(make_shared<boost::property_tree::ptree>()) {
    // Load config
    property_tree::read_json(config_filename, *m_config);
    // Connect to naoqi
    const string broker_ip = string("0.0.0.0");
    const int broker_port = 54000;
    try {
        m_broker = ALBroker::createBroker(name, broker_ip, broker_port, ip, port, 0);
    } catch (...) {
        cerr << "Fail to connect broker to: " << ip << ":" << port << std::endl;
        ALBrokerManager::getInstance()->killAllBroker();
        ALBrokerManager::kill();
        // TODO Raise exception
    }
    // Make proxies
    ALBrokerManager::setInstance(m_broker->fBrokerManager.lock());
    ALBrokerManager::getInstance()->addBroker(m_broker);
    // Devices initialisation
    m_joints = make_shared<Joints>(m_broker);
    m_leds = make_shared<LEDs>(m_broker);
    m_gyro = make_shared<Gyro>(m_broker);
    m_angle = make_shared<Angle>(m_broker);
    m_accelerometer = make_shared<Accelerometer>(m_broker);
    m_clock = make_shared<Clock>(m_broker);
    m_vision = make_shared<Vision>(*m_config);

    m_top_camera = make_shared<Camera>("/dev/video0", 320, 240, true, m_broker);
    m_bot_camera = make_shared<Camera>("/dev/video1", 320, 240, true, m_broker);
    m_top_camera->setFPS(30);
    m_bot_camera->setFPS(30);
    m_top_camera->enableCamera();
    m_bot_camera->enableCamera();

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Joints> Robot::getJoints() {
    return m_joints;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Camera> Robot::getTopCamera() {
    return m_top_camera;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Camera> Robot::getBotCamera() {
    return m_bot_camera;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<LEDs> Robot::getLEDs() {
    return m_leds;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Gyro> Robot::getGyro() {
    return m_gyro;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<Accelerometer> Robot::getAccelerometer() {
    return m_accelerometer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<Clock> Robot::getClock() {
    return m_clock;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

shared_ptr<property_tree::ptree> Robot::getConfig() {
    return m_config;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<Angle> Robot::getAngle() {
    return m_angle;
}

boost::shared_ptr<Vision> Robot::getVision() {
    return m_vision;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Robot::~Robot() {
    ALBrokerManager::getInstance()->killAllBroker();
    ALBrokerManager::kill();
}