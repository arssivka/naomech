#ifndef _Gait_h_DEFINED
#define _Gait_h_DEFINED

#include <vector>
#include <boost/shared_ptr.hpp>
#include "AbstractGait.h"
#include "GaitConstants.h"

class Gait : public AbstractGait {
public:
    Gait(const Gait& other);

    Gait(const AbstractGait& other);

    Gait(const double _stance_config[WP::LEN_STANCE_CONFIG],
         const double _step_config[WP::LEN_STEP_CONFIG],
         const double _zmp_config[WP::LEN_ZMP_CONFIG],
         const double _joint_hack_config[WP::LEN_HACK_CONFIG],
         const double _sensor_config[WP::LEN_SENSOR_CONFIG],
         const double _stiffness_config[WP::LEN_STIFF_CONFIG],
         const double _odo_config[WP::LEN_ODO_CONFIG],
         const double _arm_config[WP::LEN_ARM_CONFIG]);

protected:
    Gait();

};


static const Gait DEFAULT_GAIT = Gait(WP::STANCE_DEFAULT,
                                      WP::STEP_DEFAULT,
                                      WP::ZMP_DEFAULT,
                                      WP::HACK_DEFAULT,
                                      WP::SENSOR_DEFAULT,
                                      WP::STIFF_DEFAULT,
                                      WP::ODO_DEFAULT,
                                      WP::ARM_DEFAULT);
#endif
