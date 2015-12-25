//
// Created by arssivka on 11/30/15.
//

#include <boost/make_shared.hpp>
#include <rd/hardware/Robot.h>
#include <rd/remote/hardware/RemoteJoints.h>
#include <rd/remote/hardware/RemoteLEDs.h>
#include <rd/remote/hardware/RemoteCamera.h>
#include <rd/network/RPCServer.h>

using namespace boost;

int main() {
    rd::Robot robot("NAO", "127.0.0.1", 9559);
    rd::RPCServer srv(8080);
    srv.addModule(make_shared<rd::RemoteJoints>(robot.getJoints()));
    srv.addModule(make_shared<rd::RemoteCamera>(robot.getTopCamera(), robot.getBotCamera()));
    srv.addModule(make_shared<rd::RemoteLEDs>(robot.getLEDs()));
    srv.run();
}
