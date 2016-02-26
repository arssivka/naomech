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
    MetaGait test;

    double FAST_STANCE[] = {31.00,
                            1.45,
                            10.0,
                            5.0,
                            0.0,
                            0.1};
    double FAST_STEP[] = {0.4,
                          0.25,
                          1.1,
                          -5.0,
                          100.0,
                          -50.0,
                          100.0,
                          0.30,
                          70.0,
                          70.0,
                          0.20,
                          1.0};

    double FAST_ZMP[] = {0.0,
                         0.3,
                         4.0,
                         4.0,
                         0.01,
                         7.6};


    double FAST_SENSOR[] = {1.0,
                            0.08,
                            0.08,
                            100.0,
                            100.0,
                            7.0,
                            7.0,
                            45.0};

    double FAST_HACK[] = {5.5, 5.5};

    double FAST_STIFFNESS[] = {0.85,
                               0.3,
                               0.4,
                               0.3,
                               0.1,
                               0.1};

    double FAST_ODO[] = {1.0, 1.0, 1.0};
    double FAST_ARM[] = {0.0};

    boost::shared_ptr<rd::Kinematics> kinematics(
            make_shared<rd::Kinematics>(robot->getClock(), robot->getJoints(), robot->getConfig()));

    Gait startGait(DEFAULT_GAIT);
   // Gait startGait(FAST_STANCE, FAST_STEP, FAST_ZMP, FAST_HACK, FAST_SENSOR, FAST_STIFFNESS, FAST_ODO, FAST_ARM);
   // Gait nextGait(FAST_STANCE, FAST_STEP, FAST_ZMP, FAST_HACK, FAST_SENSOR, FAST_STIFFNESS, FAST_ODO, FAST_ARM);

    Gait nextGait(DEFAULT_GAIT);

    test.setStartGait(startGait);
    test.setNewGaitTarget(nextGait);
    StepGenerator sg(robot, &test);
    robot->getJoints()->setHardness(0.8);
    boost::shared_ptr<std::vector<double> > joints_data = sg.getDefaultStance(DEFAULT_GAIT);
    std::vector<int> keys(&StepGenerator::NB_WALKING_JOINTS[0], &StepGenerator::NB_WALKING_JOINTS[19]);
//    robot->getJoints()->setPositions(keys, *joints_data);
    //sg.setSpeed(100.0, 100.0, 0.0);
    sg.takeSteps(100.0, 0.0, 0.0, 20); // tut ustanavlivaem scolko shagov s kakimi skorostyami sdelat
    kinematics->lookAt(200.0, 0.0, 0.0, false);
    boost::shared_ptr<rd::Joints> joints = robot->getJoints();
    sleep(2);
    std::cout << "legs ticked" << std::endl;
    test.tick_gait();
    try {
        while (!sg.isDone()) { // pocka vse shagi ne sdelani (ya tak predpolagayu)
            sg.tick_controller(); // krutim frame ili tipa togo. koroche nuzhnaya shtuka
            const WalkLegsTuple& legs = sg.tick_legs(); // generireum joints i stiffnesi dlya nog
            const WalkArmsTuple& arms = sg.tick_arms(); //dlya ruk
            const std::vector<double>& lleg_joints = legs.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& rleg_joints = legs.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& lleg_gains = legs.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& rleg_gains = legs.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& larm_joints = arms.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& rarm_joints = arms.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& larm_gains = arms.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& rarm_gains = arms.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            std::copy(larm_gains.begin(), larm_gains.end(), joints_data->begin() + 0);
            std::copy(lleg_gains.begin(), lleg_gains.end(), joints_data->begin() + 4);
            std::copy(rleg_gains.begin(), rleg_gains.end(), joints_data->begin() + 10);
            std::copy(rarm_gains.begin(), rarm_gains.end(), joints_data->begin() + 16);
            joints->setHardness(keys, *joints_data);
            std::copy(larm_joints.begin(), larm_joints.end(), joints_data->begin() + 0);
            std::copy(lleg_joints.begin(), lleg_joints.end(), joints_data->begin() + 4);
            std::copy(rleg_joints.begin(), rleg_joints.end(), joints_data->begin() + 10);
            std::copy(rarm_joints.begin(), rarm_joints.end(), joints_data->begin() + 16);
            joints->setPositions(keys, *joints_data);

            usleep(10000);
        }
    } catch (...) {
        robot->getJoints()->setHardness(0.0);
    }


    sg.takeSteps(-100.0, 0.0, 0.0, 20); // tut ustanavlivaem scolko shagov s kakimi skorostyami sdelat
    //boost::shared_ptr<rd::Joints> joints = robot->getJoints();
    sleep(2);
    std::cout << "legs ticked" << std::endl;
    test.tick_gait();
    try {
        while (!sg.isDone()) { // pocka vse shagi ne sdelani (ya tak predpolagayu)
            sg.tick_controller(); // krutim frame ili tipa togo. koroche nuzhnaya shtuka
            const WalkLegsTuple& legs = sg.tick_legs(); // generireum joints i stiffnesi dlya nog
            const WalkArmsTuple& arms = sg.tick_arms(); //dlya ruk
            const std::vector<double>& lleg_joints = legs.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& rleg_joints = legs.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& lleg_gains = legs.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& rleg_gains = legs.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& larm_joints = arms.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& rarm_joints = arms.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& larm_gains = arms.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& rarm_gains = arms.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            std::copy(larm_gains.begin(), larm_gains.end(), joints_data->begin() + 0);
            std::copy(lleg_gains.begin(), lleg_gains.end(), joints_data->begin() + 4);
            std::copy(rleg_gains.begin(), rleg_gains.end(), joints_data->begin() + 10);
            std::copy(rarm_gains.begin(), rarm_gains.end(), joints_data->begin() + 16);
            joints->setHardness(keys, *joints_data);
            std::copy(larm_joints.begin(), larm_joints.end(), joints_data->begin() + 0);
            std::copy(lleg_joints.begin(), lleg_joints.end(), joints_data->begin() + 4);
            std::copy(rleg_joints.begin(), rleg_joints.end(), joints_data->begin() + 10);
            std::copy(rarm_joints.begin(), rarm_joints.end(), joints_data->begin() + 16);
            joints->setPositions(keys, *joints_data);

            usleep(10000);
        }
    } catch (...) {
        robot->getJoints()->setHardness(0.0);
    }

    sg.takeSteps(0.0, 0.0, 0.2, 20); // tut ustanavlivaem scolko shagov s kakimi skorostyami sdelat
    //boost::shared_ptr<rd::Joints> joints = robot->getJoints();
    sleep(2);
    std::cout << "legs ticked" << std::endl;
    test.tick_gait();
    try {
        while (!sg.isDone()) { // pocka vse shagi ne sdelani (ya tak predpolagayu)
            sg.tick_controller(); // krutim frame ili tipa togo. koroche nuzhnaya shtuka
            const WalkLegsTuple& legs = sg.tick_legs(); // generireum joints i stiffnesi dlya nog
            const WalkArmsTuple& arms = sg.tick_arms(); //dlya ruk
            const std::vector<double>& lleg_joints = legs.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& rleg_joints = legs.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& lleg_gains = legs.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& rleg_gains = legs.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& larm_joints = arms.get<LEFT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& rarm_joints = arms.get<RIGHT_FOOT>().get<JOINT_INDEX>();
            const std::vector<double>& larm_gains = arms.get<LEFT_FOOT>().get<STIFF_INDEX>();
            const std::vector<double>& rarm_gains = arms.get<RIGHT_FOOT>().get<STIFF_INDEX>();
            std::copy(larm_gains.begin(), larm_gains.end(), joints_data->begin() + 0);
            std::copy(lleg_gains.begin(), lleg_gains.end(), joints_data->begin() + 4);
            std::copy(rleg_gains.begin(), rleg_gains.end(), joints_data->begin() + 10);
            std::copy(rarm_gains.begin(), rarm_gains.end(), joints_data->begin() + 16);
            joints->setHardness(keys, *joints_data);
            std::copy(larm_joints.begin(), larm_joints.end(), joints_data->begin() + 0);
            std::copy(lleg_joints.begin(), lleg_joints.end(), joints_data->begin() + 4);
            std::copy(rleg_joints.begin(), rleg_joints.end(), joints_data->begin() + 10);
            std::copy(rarm_joints.begin(), rarm_joints.end(), joints_data->begin() + 16);
            joints->setPositions(keys, *joints_data);

            usleep(10000);
        }
    } catch (...) {
        robot->getJoints()->setHardness(0.0);
    }

    robot->getJoints()->setHardness(0.0);
    std::cout << "legs ticked" << std::endl;
    std::vector<double> ods = sg.getOdometryUpdate();
    for (int i = 0; i < ods.size(); ++i) {
        std::cout << "odo " << i << " " << ods[i] << std::endl;
    }
    //testing

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
