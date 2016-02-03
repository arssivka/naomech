//
// Created by arssivka on 11/30/15.
//

#include <boost/make_shared.hpp>
#include <rd/hardware/Robot.h>
#include <rd/remote/hardware/RemoteJoints.h>
#include <rd/remote/hardware/RemoteLEDs.h>
#include <rd/remote/hardware/RemoteCamera.h>
#include <rd/remote/hardware/RemoteGyro.h>
#include <rd/network/RPCServer.h>
#include <rd/remote/hardware/RemoteAccelerometer.h>
#include <rd/remote/control/RemoteKinematics.h>

using namespace boost;

int main(int argc, char* argv[]) {
    rd::Robot robot("NAO", "127.0.0.1", 9559, "../etc/naomech/resources/config.json");
    shared_ptr<rd::Kinematics> kinematics(
            make_shared<rd::Kinematics>(robot.getClock(), robot.getJoints(), robot.getConfig()));
    rd::RPCServer srv(8080);
    srv.addModule(make_shared<rd::RemoteJoints>(robot.getJoints()));
    srv.addModule(make_shared<rd::RemoteCamera>(robot.getTopCamera(), robot.getBotCamera()));
    srv.addModule(make_shared<rd::RemoteLEDs>(robot.getLEDs()));
    srv.addModule(make_shared<rd::RemoteGyro>(robot.getGyro()));
    srv.addModule(make_shared<rd::RemoteAccelerometer>(robot.getAccelerometer()));
    srv.addModule(make_shared<rd::RemoteKinematics>(kinematics));
    srv.run();
}
