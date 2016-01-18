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

using namespace boost;

int main() {
    rd::Robot robot("NAO", "127.0.0.1", 9559);
    rd::RPCServer srv(8080);
    srv.addModule(boost::make_shared<rd::RemoteJoints>(robot.getJoints()));
    srv.addModule(boost::make_shared<rd::RemoteCamera>(robot.getTopCamera(), robot.getBotCamera()));
    srv.addModule(boost::make_shared<rd::RemoteLEDs>(robot.getLEDs()));
    srv.addModule(boost::make_shared<rd::RemoteGyro>(robot.getGyro()));
    srv.addModule(boost::make_shared<rd::RemoteAccelerometer>(robot.getAccelerometer()));
    srv.run();
}
