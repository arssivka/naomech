//
// Created by arssivka on 11/23/15.
//

#include <rd/hardware/Robot.h>
#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>

using namespace AL;
using namespace boost;
using namespace std;

rd::Robot::Robot(string name, const string &ip, unsigned int port) {
    const string broker_ip = string("0.0.0.0");
    const int broker_port = 54000;
    shared_ptr<ALBroker> broker;
    try {
        broker = ALBroker::createBroker(
                name,
                broker_ip,
                broker_port,
                ip,
                port
        );
    } catch (...) {
        // TODO Log this shit!
        ALBrokerManager::getInstance()->killAllBroker();
        ALBrokerManager::kill();
        // TODO Raise exception
    }

    ALBrokerManager::setInstance(broker->fBrokerManager.lock());
    ALBrokerManager::getInstance()->addBroker(broker);

    this->dcm = make_shared<DCMProxy>(broker);
    this->mem = make_shared<ALMemoryProxy>(broker);

    this->joints = make_shared<Joints>(mem, dcm);
}

////////////////////////////////////////////////////////////////////////////////

shared_ptr<rd::Joints> rd::Robot::getJoints() {
    return this->joints;
}

////////////////////////////////////////////////////////////////////////////////

rd::Robot::~Robot() {
    AL::ALBrokerManager::getInstance()->killAllBroker();
    AL::ALBrokerManager::kill();
}
