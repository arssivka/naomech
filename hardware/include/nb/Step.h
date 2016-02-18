#ifndef Step_h_DEFINED
#define Step_h_DEFINED

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <iostream>
#include "Gait.h"


typedef boost::tuple<const double, const double, const double> distVector;

enum StepType {
    REGULAR_STEP = 0,
    //START_STEP,
            END_STEP,
    //NULL_STEP
};

enum Foot {
    LEFT_FOOT = 0,
    RIGHT_FOOT
};

struct WalkVector {
    double x;
    double y;
    double theta;
};

static const WalkVector ZERO_WALKVECTOR = {0.0, 0.0, 0.0};

/**
 * Container to hold information about steps.
 * Steps hold some arrays which contain the gait information from when they
 * were created.
 * Steps also house the logic to correctly clip steps using a few different
 * metrics
 */
class Step {
public:
    Step(const Step& other);

    Step(const WalkVector& target,
         const AbstractGait& gait,
         const Foot _foot,
         const WalkVector& last = ZERO_WALKVECTOR,
         const StepType _type = REGULAR_STEP);

    // Copy constructor to allow changing reference frames:
    Step(const double new_x, const double new_y, const double new_theta,
         const Step& other);

    void updateFrameLengths(const double duration,
                            const double dblSuppF);

    friend std::ostream& operator<<(std::ostream& o, const Step& s) {
        return o << "Step(" << s.m_x << "," << s.m_y << "," << s.theta
               << ") in " << s.m_step_config[WP::DURATION]
               << " secs with foot "
               << s.foot << " and type " << s.type;
    }

public:
    double m_x;
    double m_y;
    double theta;
    WalkVector m_walk_vector;
    unsigned int m_step_duration_frames;
    unsigned int m_double_support_frames;
    unsigned int m_single_support_frames;
    double sOffsetY;
    Foot foot;
    StepType type;
    bool zmpd;

    double m_step_config[WP::LEN_STEP_CONFIG];
    double m_zmp_config[WP::LEN_ZMP_CONFIG];
    double m_stance_config[WP::LEN_STANCE_CONFIG];
private:
    void copyGaitAttributes(const double _step_config[],
                            const double _zmp_config[],
                            const double _stance_config[]);

    void copyAttributesFromOther(const Step& other);

    void setStepSize(const WalkVector& target,
                     const WalkVector& last);

    void setStepLiftMagnitude();

    const WalkVector elipseClipVelocities(const WalkVector& source);

    const WalkVector accelClipVelocities(const WalkVector& source,
                                         const WalkVector& last);

    const WalkVector lateralClipVelocities(const WalkVector& source);
};

static const boost::shared_ptr<Step> EMPTY_STEP =
        boost::shared_ptr<Step>(new Step(ZERO_WALKVECTOR,
                                         DEFAULT_GAIT,
                                         LEFT_FOOT));
#endif
