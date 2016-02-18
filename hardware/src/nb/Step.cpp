#include <string>
#include "nb/Step.h"
#include "nb/MotionConstants.h"
#include "nb/Observer.h"

using namespace std;

//
Step::Step(const Step& other) {
    copyAttributesFromOther(other);
}

Step::Step(const WalkVector& target,
           const AbstractGait& gait, const Foot _foot,
           const WalkVector& last,
           const StepType _type)
        : m_walk_vector(target), sOffsetY(gait.stance[WP::LEG_SEPARATION_Y] * 0.5),
          foot(_foot), type(_type), zmpd(false) {
    copyGaitAttributes(gait.step, gait.zmp, gait.stance);

    switch (_type) {
        case REGULAR_STEP:
            updateFrameLengths(m_step_config[WP::DURATION], m_step_config[WP::DBL_SUPP_P]);
            break;
        case END_STEP:
            //For end steps, we are always double support, and
            //we make the length the preview period
            updateFrameLengths(static_cast<double>(Observer::NUM_PREVIEW_FRAMES) *
                               MOTION_FRAME_LENGTH_S,
                               1.0);
            break;
    }

    //After we assign elements of the gait to this step, lets clip
    setStepSize(target, last);
    setStepLiftMagnitude();
}


// Copy constructor to allow changing reference frames:
Step::Step(const double new_x, const double new_y, const double new_theta, const Step& other) {
    copyAttributesFromOther(other);
    m_x = new_x;
    m_y = new_y;
    theta = new_theta;
}

void Step::updateFrameLengths(const double duration, const double dblSuppF) {
    //need to calculate how many frames to spend in double, single
    m_step_duration_frames = static_cast<unsigned int>(duration / MOTION_FRAME_LENGTH_S);
    m_double_support_frames = static_cast<unsigned int>(duration * dblSuppF / MOTION_FRAME_LENGTH_S);
    m_single_support_frames = m_step_duration_frames - m_double_support_frames;
}


void Step::copyAttributesFromOther(const Step& other) {
    m_x = other.m_x;
    m_y = other.m_y;
    theta = other.theta;
    m_walk_vector = other.m_walk_vector;
    m_step_duration_frames = other.m_step_duration_frames;
    m_double_support_frames = other.m_double_support_frames;
    m_single_support_frames = other.m_single_support_frames;
    sOffsetY = other.sOffsetY;
    foot = other.foot;
    type = other.type;
    zmpd = other.zmpd;
    copyGaitAttributes(other.m_step_config, other.m_zmp_config, other.m_stance_config);
}

void Step::copyGaitAttributes(const double _step_config[WP::LEN_STEP_CONFIG],
                              const double _zmp_config[WP::LEN_ZMP_CONFIG],
                              const double _stance_config[WP::LEN_STANCE_CONFIG]) {
    memcpy(m_step_config, _step_config, sizeof(double) * WP::LEN_STEP_CONFIG);
    memcpy(m_zmp_config, _zmp_config, sizeof(double) * WP::LEN_ZMP_CONFIG);
    memcpy(m_stance_config, _stance_config, sizeof(double) * WP::LEN_STANCE_CONFIG);
}


void Step::setStepSize(const WalkVector& target,
                       const WalkVector& last) {

    WalkVector new_walk = ZERO_WALKVECTOR;

    new_walk = elipseClipVelocities(target);


    new_walk = accelClipVelocities(new_walk, last);
    //new_walk = target;


    m_walk_vector = new_walk; //save the walk vector for next time

    //check  if we need to clip lateral movement of this leg
    new_walk = lateralClipVelocities(new_walk);

    //Now that we have clipped the velocities, we need to convert them to distance
    //for use with this step. Note that for y and theta, we need a factor of
    //two, since you can only stafe on every other step.

    //HACK! for bacwards compatibility, the step_y is not adjusted correctly
    const double step_x = new_walk.x * m_step_config[WP::DURATION];
    const double step_y = new_walk.y * m_step_config[WP::DURATION];//*2.0;
    const double step_theta = new_walk.theta * m_step_config[WP::DURATION] * 2.0;

    //Huge architectural HACK!!!  We need to fix our transforms so we don't need to do this
    //anymore
    //This hack pmakes it possible to turn about the center of the robot, rather than the
    //center of the foot
    const double leg_sign = (foot == LEFT_FOOT ? 1.0 : -1.0);
    const double computed_x = step_x - sin(std::abs(step_theta)) * sOffsetY;
    const double computed_y = step_y + leg_sign * sOffsetY * cos(step_theta);
    const double computed_theta = step_theta;


    m_x = computed_x;
    m_y = computed_y;
    theta = computed_theta;


}


const WalkVector Step::elipseClipVelocities(const WalkVector& source) {
    // std::cout << "Ellipsoid clip input ("<<source.x<<","<<source.y
    // 			<<","<<source.theta<<")"<<std::endl;

    //Convert velocities to distances, clip them with an ellipse,
    //then convert back to velocities

    const double theta = NBMath::safe_atan2(source.y, source.x);
    const double xy_mag = std::sqrt(std::pow(source.y, 2) + std::pow(source.x, 2));

    //To find phi, the elevation out of the xy plane, we need to find a way to compare
    //radians to millimeters. The current way is to consider 'equally-weighted'
    //turning and lateral/forward motion by 'converting' the radians to something
    //more equally weighted with mm's
    const double max_xy_mag = std::sqrt(std::pow(
            m_step_config[WP::MAX_VEL_Y]
            * std::sin(theta), 2)
                                       + std::pow(
            m_step_config[WP::MAX_VEL_X]
            * std::cos(theta), 2));
    const double rad_to_mm = max_xy_mag / m_step_config[WP::MAX_VEL_THETA];
    // cout << "xy_mag = " << xy_mag << " converted theta = " << source.theta*rad_to_mm<<endl;
    const double phi = NBMath::safe_atan2(xy_mag, source.theta * rad_to_mm);

    // cout << "Ellipsoid vel. clipping: theta = "<<theta<<", phi="<<phi<<endl;

    double forward_max = 0.0;
    if (source.x > 0)
        forward_max = std::abs(m_step_config[WP::MAX_VEL_X]
                               * std::cos(theta)
                               * std::sin(phi));
    else
        forward_max = std::abs(m_step_config[WP::MIN_VEL_X]
                               * std::cos(theta)
                               * std::sin(phi));

    const double horizontal_max =
            std::abs(m_step_config[WP::MAX_VEL_Y]
                     * std::sin(theta)
                     * std::sin(phi));

    const double turning_max =
            std::abs(m_step_config[WP::MAX_VEL_THETA]
                     * std::cos(phi));
    // cout << "Clipping y="<<source.y<<" according to"<<horizontal_max<<endl;
    const double new_y_vel = NBMath::clip(source.y, horizontal_max);
    // cout << "Clipping x="<<source.x<<" according to"<<forward_max<<endl;
    const double new_x_vel = NBMath::clip(source.x, forward_max);
    // cout << "Clipping theta="<<source.theta<<" according to"<<turning_max<<endl;
    const double new_theta_vel = NBMath::clip(source.theta, turning_max);


    const WalkVector clippedVelocity = {new_x_vel, new_y_vel, new_theta_vel};
    // std::cout << "Ellipsoid clip output ("<<clippedVelocity.x<<","<<clippedVelocity.y
    // 			<<","<<clippedVelocity.theta<<")"<<std::endl;
    return clippedVelocity;
}

const WalkVector Step::accelClipVelocities(const WalkVector& source,
                                           const WalkVector& last) {
    WalkVector result = source;
    //clip velocities according to the last step, only if we aren't stopping
    if (source.x != 0.0 || source.y != 0.0 || source.theta != 0.0) {
        result.x = NBMath::clip(source.x,
                                last.x - m_step_config[WP::MAX_ACC_X],
                                last.x + m_step_config[WP::MAX_ACC_X]);
        result.y = NBMath::clip(source.y,
                                last.y - m_step_config[WP::MAX_ACC_Y] * 0.5,
                                last.y + m_step_config[WP::MAX_ACC_Y] * 0.5);
        result.theta = NBMath::clip(source.theta,
                                    last.theta - m_step_config[WP::MAX_ACC_THETA] * 0.5,
                                    last.theta + m_step_config[WP::MAX_ACC_THETA] * 0.5);
    }
    return result;
}


const WalkVector Step::lateralClipVelocities(const WalkVector& source) {
    WalkVector result = source;

    //check  if we need to clip lateral movement of this leg
    if (result.y > 0) {
        if (!foot == LEFT_FOOT) {
            result.y = 0.0;
        }
    } else if (result.y < 0) {
        if (foot == LEFT_FOOT) {
            result.y = 0.0;
        }
    }
    if (result.theta > 0) {
        if (!foot == LEFT_FOOT) {
            result.theta = 0.0;
        }
    } else if (result.theta < 0) {
        if (foot == LEFT_FOOT) {
            result.theta = 0.0;
        }
    }
    return result;
}


void Step::setStepLiftMagnitude() {

    const double percent_of_forward_max = NBMath::clip(m_x, 0, m_step_config[WP::MAX_VEL_X])
                                          / m_step_config[WP::MAX_VEL_X];

    m_step_config[WP::FOOT_LIFT_ANGLE] *= percent_of_forward_max;
}
