//
// Created by arssivka on 11/30/15.
//

#include <boost/make_shared.hpp>
#include <rd/hardware/Robot.h>
#include <rd/remote/hardware/RemoteJoints.h>
#include <rd/network/RPCServer.h>

int main() {
    rd::Robot robot("NAO", "127.0.0.1", 9559);
    rd::RPCServer srv(8080);
    srv.addModule(boost::make_shared<rd::RemoteJoints>(robot.getJoints()));
    srv.run();
}