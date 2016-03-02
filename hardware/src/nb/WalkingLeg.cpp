// This file is part of Man, a robotic perception, locomotion, and
// team strategy application created by the Northern Bites RoboCup
// team of Bowdoin College in Brunswick, Maine, for the Aldebaran
// Nao robot.
//
// Man is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Man is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser Public License for more details.
//
// You should have received a copy of the GNU General Public License
// and the GNU Lesser Public License along with Man.  If not, see
// <http://www.gnu.org/licenses/>.

#include <nb/StepGenerator.h>
#include "nb/WalkingLeg.h"
#include "rd/hardware/Robot.h"
#include "nb/COMKinematics.h"

using namespace std;

using namespace Kinematics;
using namespace NBMath;

//
WalkingLeg::WalkingLeg(boost::shared_ptr<rd::Robot> robot,
                       boost::shared_ptr<SensorAngles> sensor_angles,
                       const MetaGait* _gait,
                       ChainID id)
        : m_keys(&StepGenerator::NB_WALKING_JOINTS[0], &StepGenerator::NB_WALKING_JOINTS[19]),
          m_robot(robot),
          m_sensor_angles(sensor_angles),
          state(SUPPORTING),
          frameCounter(0),
          cur_dest(EMPTY_STEP), swing_src(EMPTY_STEP), swing_dest(EMPTY_STEP),
          support_step(EMPTY_STEP),
          chainID(id), gait(_gait),
          goal(CoordFrame3D::vector3D(0.0, 0.0, 0.0)),
          last_goal(CoordFrame3D::vector3D(0.0, 0.0, 0.0)),
          lastRotation(0.0), odoUpdate(3, 0.0),
          leg_sign(id == LLEG_CHAIN ? 1 : -1),
          leg_name(id == LLEG_CHAIN ? "left" : "right"),
          m_sensor_angle_x(0.0), m_sensor_angle_y(0.0) {


    for (unsigned int i = 0; i < LEG_JOINTS; i++) lastJoints[i] = 0.0;
}


WalkingLeg::~WalkingLeg() {


}

void WalkingLeg::setSteps(boost::shared_ptr<Step> _swing_src,
                          boost::shared_ptr<Step> _swing_dest,
                          boost::shared_ptr<Step> _support_step) {
    swing_src = _swing_src;
    swing_dest = _swing_dest;
    support_step = _support_step;
    assignStateTimes(support_step);
}

LegJointStiffTuple WalkingLeg::tick(boost::shared_ptr<Step> step,
                                    boost::shared_ptr<Step> _swing_src,
                                    boost::shared_ptr<Step> _swing_dest,
                                    ufmatrix3 fc_Transform) {

    cur_dest = step;
    swing_src = _swing_src;
    swing_dest = _swing_dest;

    //ufvector3 dest_f = CoordFrame3D::vector3D(cur_dest->x,cur_dest->y);
    //ufvector3 dest_c = prod(fc_Transform,dest_f);
    //double dest_x = dest_c(0);
    //double dest_y = dest_c(1);
    LegJointStiffTuple result;
    switch (state) {
        case SUPPORTING:
            result = supporting(fc_Transform);
            break;
        case SWINGING:
            result = swinging(fc_Transform);
            break;
        case DOUBLE_SUPPORT:
            //In dbl sup, we have already got the final target after swinging in
            //mind, so we actually want to keep the target as the "source"
            cur_dest = swing_src;
            result = supporting(fc_Transform);
            break;
        case PERSISTENT_DOUBLE_SUPPORT:
            result = supporting(fc_Transform);
            break;
        default:
            cout << "Invalid SupportMode" << endl;
            throw "Invalid SupportMode passed to WalkingLeg::tick";
    }

    computeOdoUpdate();

    last_goal = goal;
    lastRotation = getFootRotation();
    frameCounter++;
    //Decide if it's time to switch states

    //Decide if we need to switch states. Run twice in case state time of new
    //state is zero 
    for (unsigned int i = 0; shouldSwitchStates() && i < 2; i++, switchToNextState());

    return result;
}
//#define SENSOR_SCALE 0.75
#define SENSOR_SCALE 0.0

LegJointStiffTuple WalkingLeg::swinging(ufmatrix3 fc_Transform) {
    ufvector3 dest_f = CoordFrame3D::vector3D(cur_dest->m_x, cur_dest->m_y);
    ufvector3 src_f = CoordFrame3D::vector3D(swing_src->m_x, swing_src->m_y);

    ufvector3 dest_c = prod(fc_Transform, dest_f);
    ufvector3 src_c = prod(fc_Transform, src_f);

    //double dest_x = dest_c(0);
    //double dest_y = dest_c(1);

    static double dist_to_cover_x = 0;
    static double dist_to_cover_y = 0;

    if (firstFrame()) {
        dist_to_cover_x = cur_dest->m_x - swing_src->m_x;
        dist_to_cover_y = cur_dest->m_y - swing_src->m_y;
    }

    //There are two attirbutes to control - the height off the ground, and
    //the progress towards the goal.

    //HORIZONTAL PROGRESS:
    double percent_complete = (static_cast<double>(frameCounter) / static_cast<double>(singleSupportFrames));

    double theta = percent_complete * 2.0 * M_PI_double;
    double stepHeight = gait->step[WP::STEP_HEIGHT];
    double percent_to_dest_horizontal = NBMath::cycloidx(theta) / (2.0 * M_PI_double);

    //Then we can express the destination as the proportionate distance to cover
    double dest_x = src_f(0) + percent_to_dest_horizontal * dist_to_cover_x;
    double dest_y = src_f(1) + percent_to_dest_horizontal * dist_to_cover_y;

    ufvector3 target_f = CoordFrame3D::vector3D(dest_x, dest_y);
    ufvector3 target_c = prod(fc_Transform, target_f);

    double target_c_x = target_c(0);
    double target_c_y = target_c(1);

    double radius = gait->step[WP::STEP_HEIGHT] / 2;
    double heightOffGround = radius * NBMath::cycloidy(theta);

    goal(0) = target_c_x;
    goal(1) = target_c_y;
    goal(2) = -gait->stance[WP::BODY_HEIGHT] + heightOffGround;


    vector<double> joint_result = finalizeJoints(goal);

    vector<double> stiff_result = getStiffnesses();
    return LegJointStiffTuple(joint_result, stiff_result);
}

LegJointStiffTuple WalkingLeg::supporting(ufmatrix3 fc_Transform) {//double dest_x, double dest_y) {
    /**
       this method calculates the angles for this leg when it is on the ground
       (i.e. the leg on the ground in single support, or either leg in double
       support).
       We calculate the goal based on the comx,comy from the controller,
       and the given parameters using inverse kinematics.
     */
    ufvector3 dest_f = CoordFrame3D::vector3D(cur_dest->m_x, cur_dest->m_y);
    ufvector3 dest_c = prod(fc_Transform, dest_f);
    double dest_x = dest_c(0);
    double dest_y = dest_c(1);

    double physicalHipOffY = 0;
    goal(0) = dest_x; //targetX for this leg
    goal(1) = dest_y;  //targetY
    goal(2) = -gait->stance[WP::BODY_HEIGHT];         //targetZ

    vector<double> joint_result = finalizeJoints(goal);
    vector<double> stiff_result = getStiffnesses();
    return LegJointStiffTuple(joint_result, stiff_result);
}


const vector<double> WalkingLeg::finalizeJoints(const ufvector3& footGoal) {
    const double startStopSensorScale = getEndStepSensorScale();


    //Center of mass control
#ifdef USE_COM_CONTROL
    const double COM_SCALE = startStopSensorScale;
    const ufvector4 com_c = Kinematics::getCOMc(m_robot->getJoints()->getPositions(m_keys)->data);
#else
    const double COM_SCALE = startStopSensorScale;
    const ufvector4 com_c = CoordFrame4D::vector4D(0, 0, 0);
#endif

    //HACK -- the startStopSensor gives us a nice in/out scaling from motion
    //we should really rename that function
    const double COM_Z_OFF = 69.9;
    ufvector3 comFootGoal = footGoal;
    comFootGoal(2) += COM_Z_OFF * COM_SCALE;

    const boost::tuple <const float, const float > sensorCompensation =
            m_sensor_angles->getAngles(startStopSensorScale);

    const double bodyAngleX = m_sensor_angle_x = sensorCompensation.get<SensorAngles::X>();
    const double bodyAngleY = m_sensor_angle_y = sensorCompensation.get<SensorAngles::Y>();

    //Hack
    const boost::tuple<const double, const double> ankleAngleCompensation = getAnkleAngles();

    const double footAngleX = ankleAngleCompensation.get<0>();
    const double footAngleY = ankleAngleCompensation.get<1>();
    const double footAngleZ = getFootRotation_c() + leg_sign * gait->stance[WP::LEG_ROT_Z] * 0.5;

    const ufvector3 bodyOrientation = CoordFrame3D::vector3D(bodyAngleX, bodyAngleY, 0.0);
    const ufvector3 footOrientation = CoordFrame3D::vector3D(footAngleX, footAngleY, footAngleZ);


    const ufvector3 bodyGoal = CoordFrame3D::vector3D(-com_c(0) * COM_SCALE, -com_c(1) * COM_SCALE, COM_Z_OFF * COM_SCALE);


    IKLegResult result = Kinematics::legIK(chainID, comFootGoal, footOrientation, bodyGoal, bodyOrientation);

    applyHipHacks(result.angles);

    memcpy(lastJoints, result.angles, LEG_JOINTS * sizeof(double));
    return vector<double>(result.angles, &result.angles[LEG_JOINTS]);

}

const boost::tuple<const double, const double>
WalkingLeg::getAnkleAngles() {
    if (state != SWINGING) {
        return boost::tuple<const double, const double>(0.0, 0.0);
    }

    const double angle = static_cast<double>(frameCounter) / static_cast<double>(singleSupportFrames) * M_PI_double;

    const double scale = std::sin(angle);


    const double ANKLE_LIFT_ANGLE = swing_dest->m_step_config[WP::FOOT_LIFT_ANGLE] * scale;

    return boost::tuple<const double, const double>(0.0, ANKLE_LIFT_ANGLE);
}


/*
 * When we are starting and stopping, we want to gradually turn on or off sensor
 * feedback.  We do this by checking if the support step is an END step,
 * and to see if the next step is also an end step. If so, and the next one
 * is an end step as well, then we are stopping
 * if not, then we are starting
 */
const double WalkingLeg::getEndStepSensorScale() {

    //do nothing for regular steps
    if (support_step->type == REGULAR_STEP)
        return 1.0;

    if (swing_src->type == END_STEP)
        //We've already stopped, so don't scale down again!
        return 0.0;

    double startScale, endScale;
    if (swing_dest->type == REGULAR_STEP) {
        //Starting from stopped
        startScale = 0.0;
        endScale = 1.0;
    } else {
        //Stopping
        startScale = 1.0;
        endScale = 0.0;
    }
    //WARNING: Assume in an END step, all frames of cycle are double support
    double percent_complete = static_cast<double>(frameCounter) / static_cast<double>(doubleSupportFrames);

    const double theta = percent_complete * 2.0 * M_PI_double;//TODO: move to common
    const double percent_to_dest = NBMath::cycloidx(theta) / (2.0 * M_PI_double);

    return startScale + (endScale - startScale) * percent_to_dest;
}

/*  Returns the rotation for this motion frame which we expect
 *  the swinging foot to have relative to the support foot (in the f frame)
 */
const double WalkingLeg::getFootRotation() {
    if (state != SUPPORTING && state != SWINGING)
        return swing_src->theta;

    const double percent_complete = static_cast<double>(frameCounter) / static_cast<double>(singleSupportFrames);

    const double theta = percent_complete * 2.0 * M_PI_double;
    const double percent_to_dest = NBMath::cycloidx(theta) / (2.0 * M_PI_double);

    const double end = swing_dest->theta;
    const double start = swing_src->theta;

    const double value = start + (end - start) * percent_to_dest;
    return value;
}

/* Returns the foot rotation for this foot relative to the C frame*/
const double WalkingLeg::getFootRotation_c() {
    const double abs_rot = std::abs(getFootRotation());
    const double rot_rel_c = abs_rot * 0.5 * leg_sign;
    return rot_rel_c;
}

/* We assume!!! that the rotation of the hip yaw pitch joint should be 1/2*/
const double WalkingLeg::getHipYawPitch() {
    return -fabs(getFootRotation() * 0.5);
}

void WalkingLeg::applyHipHacks(double angles[]) {
    const double footAngleZ = getFootRotation_c();
    boost::tuple<const double, const double> hipHacks = getHipHack(footAngleZ);
    angles[1] += hipHacks.get<1>(); //HipRoll
    angles[2] += hipHacks.get<0>(); //HipPitch
}

/**
 * Function returns the angle to add to the hip roll joint depending on
 * how far along we are in the process of a state (namely swinging,
 * and supporting)
 *
 * In certain circumstances, this function returns 0.0, since we don't
 * want to be hacking the hio when we are starting and stopping.
 */
const boost::tuple<const double, const double> WalkingLeg::getHipHack(const double footAngleZ) {
    ChainID hack_chain;
    if (state == SUPPORTING) {
        hack_chain = chainID;
    } else if (state == SWINGING) {
        hack_chain = getOtherLegChainID();
    } else {
        // This step is double support, returning 0 hip hack
        return 0.0;
    }
    const double support_sign = (state != SWINGING ? 1.0 : -1.0);
    const double absFootAngle = std::abs(footAngleZ);


    //Calculate the compensation to the HIPROLL
    double MAX_HIP_ANGLE_OFFSET = (hack_chain == LLEG_CHAIN ?
                                  gait->hack[WP::L_HIP_AMP] :
                                  gait->hack[WP::R_HIP_AMP]);

    // the swinging leg will follow a trapezoid in 3-d. The trapezoid has
    // three stages: going up, a level stretch, going back down to the ground
    static int stage;
    if (firstFrame()) stage = 0;

    double hr_offset = 0.0;

    if (stage == 0) { // we are rising
        // we want to raise the foot up for the first third of the step duration
        hr_offset = MAX_HIP_ANGLE_OFFSET * static_cast<double>(frameCounter) / (static_cast<double>(singleSupportFrames) / 3.0);
        if (frameCounter >= (static_cast<double>(singleSupportFrames) / 3.0)) stage++;

    }
    else if (stage == 1) { // keep it level
        hr_offset = MAX_HIP_ANGLE_OFFSET;
        if (frameCounter >= 2. * static_cast<double>(singleSupportFrames) / 3)
            stage++;
    }
    else {// stage 2, set the foot back down on the ground
        hr_offset = max(0.0, MAX_HIP_ANGLE_OFFSET * static_cast<double>(singleSupportFrames - frameCounter) /
                        (static_cast<double>(singleSupportFrames) / 3.0));
    }

    //we've calcuated the correct magnitude, but need to adjust for specific
    //hip motor angle direction in this leg
    //AND we also need to rotate some of the correction to the hip pitch motor
    // (This is kind of a HACK until we move the step lifting to be taken
    // directly into account when we determine x,y 3d targets for each leg)
    const double hipPitchAdjustment = -hr_offset * std::sin(footAngleZ);
    const double hipRollAdjustment = support_sign * (hr_offset * static_cast<double>(leg_sign) * std::cos(footAngleZ));

    return boost::tuple<const double, const double>(hipPitchAdjustment, hipRollAdjustment);
    //return leg_sign*hr_offset;
}

/**
 * Determine the stiffness for all the joints in the leg at the current point
 * in the gait cycle. Currently, the stiffnesses are static throughout the gait
 * cycle
 */
const vector<double> WalkingLeg::getStiffnesses() {

    //get shorter names for all the constants
    const double maxS = gait->stiffness[WP::HIP];
    const double anklePitchS = gait->stiffness[WP::AP];
    const double ankleRollS = gait->stiffness[WP::AR];
    const double kneeS = gait->stiffness[WP::KP];

    double stiffnesses[LEG_JOINTS] = {maxS, maxS, maxS, kneeS, anklePitchS, ankleRollS};
    vector<double> stiff_result = vector<double>(stiffnesses, &stiffnesses[LEG_JOINTS]);
    return stiff_result;

}


inline ChainID WalkingLeg::getOtherLegChainID() {
    return (chainID == LLEG_CHAIN ?
            RLEG_CHAIN : LLEG_CHAIN);
}


void WalkingLeg::computeOdoUpdate() {
    const double thetaDiff = getFootRotation() - lastRotation;
    //TODO: add a odometry calibration section to walkParams
    const double thetaCOMMovement = -thetaDiff * 0.33; //.33 is somewhat experimental

    const ufvector3 diff = goal - last_goal;
    const double xCOMMovement = -diff(0);
    const double yCOMMovement = -diff(1);

    odoUpdate[0] = xCOMMovement * gait->odo[WP::X_SCALE];
    odoUpdate[1] = yCOMMovement * gait->odo[WP::Y_SCALE];
    odoUpdate[2] = thetaCOMMovement * gait->odo[WP::THETA_SCALE];
}

/**
 *  STATIC!! method to get angles from a goal, and the components of walking params
 */
vector<double> WalkingLeg::getAnglesFromGoal(const ChainID chainID,
                              const ufvector3& goal,
                              const double stance[WP::LEN_STANCE_CONFIG]) {

    const double sign = (chainID == LLEG_CHAIN ? 1.0 : -1.0);

    const ufvector3 body_orientation = CoordFrame3D::vector3D(0.0, stance[WP::BODY_ROT_Y], 0.0);
    const ufvector3 foot_orientation = CoordFrame3D::vector3D(0.0, 0.0, sign * stance[WP::LEG_ROT_Z] * 0.5);
    const ufvector3 body_goal = CoordFrame3D::vector3D(0.0, 0.0, 0.0);


    IKLegResult result = Kinematics::legIK(chainID, goal, foot_orientation, body_goal, body_orientation);
    return vector<double>(result.angles, &result.angles[LEG_JOINTS]);

}


/**
 * Assuming this is the support foot, then we can return how far we have moved
 */
vector<double> WalkingLeg::getOdoUpdate() {
    return odoUpdate;
}

void WalkingLeg::startLeft() {
    if (chainID == LLEG_CHAIN) {
        //we will start walking first by swinging left leg (this leg), so
        //we want double, not persistent, support
        setState(DOUBLE_SUPPORT);
    } else {
        setState(PERSISTENT_DOUBLE_SUPPORT);
    }
}

void WalkingLeg::startRight() {
    if (chainID == LLEG_CHAIN) {
        //we will start walking first by swinging right leg (not this leg), so
        //we want persistent double support
        setState(PERSISTENT_DOUBLE_SUPPORT);
    } else {
        setState(DOUBLE_SUPPORT);
    }
}


//transition function of the fsa
//returns the next logical state to enter
SupportMode WalkingLeg::nextState() {
    switch (state) {
        case SUPPORTING:
            return DOUBLE_SUPPORT;
        case SWINGING:
            return PERSISTENT_DOUBLE_SUPPORT;
        case DOUBLE_SUPPORT:
            return SWINGING;
        case PERSISTENT_DOUBLE_SUPPORT:
            return SUPPORTING;
        default:
            throw "Non existent state";
            return PERSISTENT_DOUBLE_SUPPORT;
    }
}

//Decides if time is up for the current state
bool WalkingLeg::shouldSwitchStates() {
    switch (state) {
        case SUPPORTING:
            return frameCounter >= singleSupportFrames;
        case SWINGING:
            return frameCounter >= singleSupportFrames;
        case DOUBLE_SUPPORT:
            return frameCounter >= doubleSupportFrames;
        case PERSISTENT_DOUBLE_SUPPORT:
            return frameCounter >= doubleSupportFrames;
    }

    throw "Non existent state";
    return false;
}

void WalkingLeg::switchToNextState() {
    setState(nextState());
}

void WalkingLeg::setState(SupportMode newState) {
    state = newState;
    frameCounter = 0;
    if (state == PERSISTENT_DOUBLE_SUPPORT ||
        state == DOUBLE_SUPPORT)
        lastRotation = -lastRotation;
}

void WalkingLeg::assignStateTimes(boost::shared_ptr<Step> step) {
    doubleSupportFrames = step->m_double_support_frames;
    singleSupportFrames = step->m_single_support_frames;
    cycleFrames = step->m_step_duration_frames;
}
