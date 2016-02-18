#ifndef AbstractGait_h_DEFINED
#define AbstractGait_h_DEFINED

#include <string>

#include "GaitConstants.h"

class AbstractGait {
public:
    AbstractGait();

    virtual ~AbstractGait();

    std::string toString() const;

protected:
    void setGaitFromGait(const AbstractGait& other);

    void setGaitFromArrays(const double _stance_config[WP::LEN_STANCE_CONFIG],
                           const double _step_config[WP::LEN_STEP_CONFIG],
                           const double _zmp_config[WP::LEN_ZMP_CONFIG],
                           const double _joint_hack_config[WP::LEN_HACK_CONFIG],
                           const double _sensor_config[WP::LEN_SENSOR_CONFIG],
                           const double _stiffness_config[WP::LEN_STIFF_CONFIG],
                           const double _odo_config[WP::LEN_ODO_CONFIG],
                           const double _arm_config[WP::LEN_ARM_CONFIG]);


    template<const unsigned int length>
    static void addSubComponent(double target[length],
                                const double array1[length],
                                const double array2[length]);

    template<const unsigned int length>
    static void multiplySubComponent(double target[length],
                                     const double source[length],
                                     const double scalar);

    static void interpolateGaits(AbstractGait& targetGait,
                                 const AbstractGait& startGait,
                                 const AbstractGait& endGait,
                                 const double percentComplete);

    template<const unsigned int length>
    static void
            combineSubComponents(double target[length],
                                 const double source1[length],
                                 const double source2[length],
                                 const double percentSwitched);

public:
    double stance[WP::LEN_STANCE_CONFIG],
            step[WP::LEN_STEP_CONFIG],
            zmp[WP::LEN_ZMP_CONFIG],
            hack[WP::LEN_HACK_CONFIG],
            sensor[WP::LEN_SENSOR_CONFIG],
            stiffness[WP::LEN_STIFF_CONFIG],
            odo[WP::LEN_ODO_CONFIG],
            arm[WP::LEN_ARM_CONFIG];

};

#endif
