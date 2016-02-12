//
// Created by arssivka on 11/30/15.
//

#include <boost/make_shared.hpp>
#include <rd/hardware/Robot.h>
#include <rd/hardware/RemoteJoints.h>
#include <rd/hardware/RemoteLEDs.h>
#include <rd/hardware/RemoteCamera.h>
#include <rd/hardware/RemoteGyro.h>
#include <rd/network/RPCServer.h>
#include <rd/hardware/RemoteAccelerometer.h>
#include <rd/hardware/RemoteKinematics.h>
#include <boost/program_options.hpp>
#include <rd/hardware/RemoteAngle.h>
///testing
#include <nb/StepGenerator.h>
#include <nb/MetaGait.h>
#include <vector>
#include <nb/Gait.h>

using namespace boost;
namespace po = boost::program_options;

int main(int argc, const char* const argv[]) {
    // Server parameter variables
    std::string ip;
    int port;
    std::string config_file;
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help,h", "produce help message")
            ("ip", po::value<std::string>(&ip)->default_value("127.0.0.1"), "set server ip")
            ("port", po::value<int>(&port)->default_value(5469), "set server port")
            ("config", po::value<std::string>(), "path to configuration file");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    if (vm.count("config")) {
        config_file = vm["config"].as<std::string>();
    } else {
        std::cout << "Configuration file was not set" << std::endl;
        return EXIT_FAILURE;
    }


    boost::shared_ptr<rd::Robot> robot = boost::make_shared<rd::Robot>("NAO", ip, 9559, config_file);
    //testing
    MetaGait *test;
    StepGenerator sg(robot, test);
    robot->getJoints()->setHardness(0.8);
    std::vector<float>* allJoints;
    allJoints = sg.getDefaultStance(DEFAULT_GAIT);
    std::vector<double> allJointsDoubles(allJoints->begin(), allJoints->end());
    robot->getJoints()->setPositions(robot->getJoints()->getKeys(), allJointsDoubles);
    //sg.setSpeed(100.0, 100.0, 0.0);
    sg.takeSteps(100.0, 100.0, 0.0, 10); // tut ustanavlivaem scolko shagov s kakimi skorostyami sdelat
    robot->getJoints()->setHardness(0.0);
    std::cout << "legs ticked" << std::endl;
    while (!sg.isDone()) { // pocka vse shagi ne sdelani (ya tak predpolagayu)
        sg.tick_legs(); // generireum joints i stiffnesi dlya nog
        sg.tick_arms(); //dlya ruk
        sg.tick_controller(); // krutim frame ili tipa togo. koroche nuzhnaya shtuka
    }
    std::cout << "legs ticked" << std::endl;
    //testing
    shared_ptr<rd::Kinematics> kinematics(
            make_shared<rd::Kinematics>(robot->getClock(), robot->getJoints(), robot->getConfig()));
    rd::RPCServer srv(port);
    srv.addModule(make_shared<rd::RemoteJoints>(robot->getJoints()));
    srv.addModule(make_shared<rd::RemoteCamera>(robot->getTopCamera(), robot->getBotCamera()));
    srv.addModule(make_shared<rd::RemoteLEDs>(robot->getLEDs()));
    srv.addModule(make_shared<rd::RemoteGyro>(robot->getGyro()));
    srv.addModule(make_shared<rd::RemoteAccelerometer>(robot->getAccelerometer()));
    srv.addModule(make_shared<rd::RemoteAngle>(robot->getAngle()));
    srv.addModule(make_shared<rd::RemoteKinematics>(kinematics));
    srv.run();
    return EXIT_SUCCESS;
}
