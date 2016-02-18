#include "nb/Gait.h"
#include <iostream>

using namespace std;

Gait::Gait(const Gait& other) {
    setGaitFromGait(other);
    //cout << "from Gait: "<<endl<< toString() <<endl;

}

Gait::Gait(const AbstractGait& other) {
    setGaitFromGait(other);
    //cout << "from Abstract gait: "<<endl<< toString() <<endl;
}

Gait::Gait(
        const double _stance_config[WP::LEN_STANCE_CONFIG],
        const double _step_config[WP::LEN_STEP_CONFIG],
        const double _zmp_config[WP::LEN_ZMP_CONFIG],
        const double _joint_hack_config[WP::LEN_HACK_CONFIG],
        const double _sensor_config[WP::LEN_SENSOR_CONFIG],
        const double _stiffness_config[WP::LEN_STIFF_CONFIG],
        const double _odo_config[WP::LEN_ODO_CONFIG],
        const double _arm_config[WP::LEN_ARM_CONFIG]) {
    setGaitFromArrays(_stance_config,
                      _step_config,
                      _zmp_config,
                      _joint_hack_config,
                      _sensor_config,
                      _stiffness_config,
                      _odo_config,
                      _arm_config);
    //cout << "from arrays: "<<endl<< toString() <<endl;
}


//Default constructor - use at your own risk
Gait::Gait() {
    std::cout << "Danger Mr. Robinson - are you sure you want to default"
            " construct walk parameters?" << std::endl;
}
