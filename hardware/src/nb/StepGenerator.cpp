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

#include <iostream>

using namespace std;

#include <boost/shared_ptr.hpp>
#include <boost/assign/std/vector.hpp>

using namespace boost::assign;
using boost::shared_ptr;

#include <nb/StepGenerator.h>
#include "nb/Observer.h"

using namespace boost::numeric;
using namespace NBMath;
using namespace Kinematics;

int StepGenerator::NB_WALKING_JOINTS[20] = {
        rd::Joints::L_SHOULDER_PITCH,
        rd::Joints::L_SHOULDER_ROLL,
        rd::Joints::L_ELBOW_YAW,
        rd::Joints::L_ELBOW_ROLL,
        rd::Joints::L_HIP_YAW_PITCH,
        rd::Joints::L_HIP_ROLL,
        rd::Joints::L_HIP_PITCH,
        rd::Joints::L_KNEE_PITCH,
        rd::Joints::L_ANKLE_PITCH,
        rd::Joints::L_ANKLE_ROLL,
        rd::Joints::R_HIP_YAW_PITCH,
        rd::Joints::R_HIP_ROLL,
        rd::Joints::R_HIP_PITCH,
        rd::Joints::R_KNEE_PITCH,
        rd::Joints::R_ANKLE_PITCH,
        rd::Joints::R_ANKLE_ROLL,
        rd::Joints::R_SHOULDER_PITCH,
        rd::Joints::R_SHOULDER_ROLL,
        rd::Joints::R_ELBOW_YAW,
        rd::Joints::R_ELBOW_ROLL
};


StepGenerator::StepGenerator(shared_ptr<rd::Robot> robot, const MetaGait* _gait)
        : m_x(0.0), m_y(0.0), m_theta(0.0),
          m_done(true),
          m_sensor_angles(boost::make_shared<SensorAngles>(robot, _gait)),
          m_com_i(CoordFrame3D::vector3D(0.0, 0.0)),
          m_com_f(CoordFrame3D::vector3D(0.0, 0.0)),
          m_est_zmp_i(CoordFrame3D::vector3D(0.0, 0.0)),
          m_zmp_ref_x(list<double>()), m_zmp_ref_y(list<double>()), futureSteps(),
          currentZMPDSteps(),
          si_Transform(CoordFrame3D::identity3D()),
          m_last_zmp_end_s(CoordFrame3D::vector3D(0.0, 0.0)),
          m_if_transform(CoordFrame3D::identity3D()),
          m_fc_transform(CoordFrame3D::identity3D()),
          cc_Transform(CoordFrame3D::identity3D()),
          m_robot(robot), m_gait(_gait), m_next_step_is_feft(true), waitForController(0),
          leftLeg(robot, m_sensor_angles, m_gait, LLEG_CHAIN),
          rightLeg(robot, m_sensor_angles, m_gait, RLEG_CHAIN),
          leftArm(m_gait, LARM_CHAIN), rightArm(m_gait, RARM_CHAIN),
          supportFoot(LEFT_SUPPORT),
          m_controller_x(new Observer()),
          m_controller_y(new Observer()),
          m_zmp_filter(),
          m_acc_filter(),
          mAccInWorldFrame(CoordFrame4D::vector4D(0.0, 0.0, 0.0)) { }

StepGenerator::~StepGenerator() {
    delete m_controller_x;
    delete m_controller_y;
}

void StepGenerator::resetHard() {
    //When we fall, or other bad things happen, we just want to stop asap
    //Clear all zmp, clear footsteps
    resetQueues();
    m_done = true;
    //I don't think there is much else we need to do....
}


/**
 * Central method to get the previewed zmp_refernce values
 * In the process of getting these values, this method handles the following:    80
 *
 *  * Handles transfer from futureSteps list to the currentZMPDsteps list.
 *    When the Future ZMP values we want run out, we pop the next future step
 *    add generated ZMP from it, and put it into the ZMPDsteps List
 *
 *  * Ensures that there are NUM_PREVIEW_FRAMES + 1 frames in the zmp lists.
 *    the oldest value will be popped off before the list is sent to the
 *    controller.
 *
 */
zmp_xy_tuple StepGenerator::generate_zmp_ref() {
    //Generate enough ZMPs so a) the controller can run
    //and                     b) there are enough steps
    while (m_zmp_ref_y.size() <= Observer::NUM_PREVIEW_FRAMES ||
           // VERY IMPORTANT: make sure we have enough ZMPed steps
           currentZMPDSteps.size() < MIN_NUM_ENQUEUED_STEPS) {
        if (futureSteps.size() == 0) {
            generateStep(m_x, m_y, m_theta); // replenish with the current walk vector
        } else {
            shared_ptr<Step> nextStep = futureSteps.front();
            futureSteps.pop_front();
            fillZMP(nextStep);
            //transfer the nextStep element from future to current list
            currentZMPDSteps.push_back(nextStep);
        }
    }
    return zmp_xy_tuple(&m_zmp_ref_x, &m_zmp_ref_y);
}

double StepGenerator::scaleSensors(const double sensorZMP, const double perfectZMP) {
    const double sensorWeight = 0.0;//gait->sensor[WP::OBSERVER_SCALE];
    return sensorZMP * sensorWeight + (1.0 - sensorWeight) * perfectZMP;
}

/**
 *  This method gets called by the walk provider to generate the output
 *  of the ZMP controller. It generates the com location in the I frame
 */
void StepGenerator::tick_controller() {


    //Turned off sensor zmp for now since we have a better method
    //JS June 2009
    //findSensorZMP();

    zmp_xy_tuple zmp_ref = generate_zmp_ref();

    //The observer needs to know the current reference zmp
    const double cur_zmp_ref_x = m_zmp_ref_x.front();
    const double cur_zmp_ref_y = m_zmp_ref_y.front();
    //clear the oldest (i.e. current) value from the preview list
    m_zmp_ref_x.pop_front();
    m_zmp_ref_y.pop_front();

    //Scale the sensor feedback according to the gait parameters
    m_est_zmp_i(0) = scaleSensors(m_zmp_filter.get_zmp_x(), cur_zmp_ref_x);
    m_est_zmp_i(1) = scaleSensors(m_zmp_filter.get_zmp_y(), cur_zmp_ref_y);

    //Tick the controller (input: ZMPref, sensors -- out: CoM x, y)

    const double com_x = m_controller_x->tick(zmp_ref.get<0>(), cur_zmp_ref_x, m_est_zmp_i(0));
    /*
    // TODO! for now we are disabling the observer for the x direction
    // by reporting a sensor zmp equal to the planned/expected value
    const double com_x = m_controller_x->tick(zmp_ref.get<0>(),cur_zmp_ref_x,
                                           cur_zmp_ref_x); // NOTE!
    */
    const double com_y = m_controller_y->tick(zmp_ref.get<1>(), cur_zmp_ref_y, m_est_zmp_i(1));
    m_com_i = CoordFrame3D::vector3D(com_x, com_y);

}

/** Central method for moving the walking legs. It handles important stuff like:
 *
 *  * Switching support feet
 *  * Updating the coordinate transformations from i to f and f to c
 *  * Keep running track of the f locations of the following 'footholds':
 *    - the support foot (supp_pos_f), which depends on the first step in the
 *      ZMPDSteps queue, but is always at the origin in f frame, by definition
 *    - the source of the swinging foot (swing_src_f), which is the location
 *      where the now swinging foot was once the supporting foot
 *    - the destination of the swinging foot (swing_pos_f), which depends on
 *      the second step in the ZMPDSteps queue
 *  * Handles poping from the ZMPDStep list when we switch support feet
 */


WalkLegsTuple StepGenerator::tick_legs() {
    m_sensor_angles->tick_sensors();

    //Decide if this is the first frame into any double support phase
    //which is the critical point when we must swap coord frames, etc
    if (leftLeg.isSwitchingSupportMode() && leftLeg.stateIsDoubleSupport()) {
        swapSupportLegs();
    }

    //std::cout << "Support step is " << *supportStep_f <<std::endl;
    //hack-ish for now to do hyp pitch crap
    leftLeg.setSteps(m_swinging_step_source_f, m_swinging_step_f, m_support_step_f);
    rightLeg.setSteps(m_swinging_step_source_f, m_swinging_step_f, m_support_step_f);

    //Each frame, we must recalculate the location of the center of mass
    //relative to the support leg (f coord frame), based on the output
    //of the controller (in tick_controller() )
    m_com_f = prod(m_if_transform, m_com_i);

    //We want to get the incremental rotation of the center of mass
    //we need to ask one of the walking legs to give it:
    const double body_rot_angle_fc = leftLeg.getFootRotation() / 2; //relative to f

    //Using the location of the com in the f coord frame, we can calculate
    //a transformation matrix to go from f to c
    m_fc_transform = prod(CoordFrame3D::rotation3D(CoordFrame3D::Z_AXIS, body_rot_angle_fc),
                          CoordFrame3D::translation3D(-m_com_f(0), -m_com_f(1)));

    //Now we need to determine which leg to send the coorect footholds/Steps to
    shared_ptr<Step> leftStep_f, rightStep_f;
    //First, the support leg.
    if (m_support_step_f->foot == LEFT_FOOT) {
        leftStep_f = m_support_step_f;
        rightStep_f = m_swinging_step_f;
    }
    else {
        rightStep_f = m_support_step_f;
        leftStep_f = m_swinging_step_f;
    }

    //Since we'd like to ignore the state information of the WalkinLeg as much
    //as possible, we send in the source of the swinging leg to both, regardless
    LegJointStiffTuple left = leftLeg.tick(leftStep_f, m_swinging_step_source_f, m_swinging_step_f, m_fc_transform);
    LegJointStiffTuple right = rightLeg.tick(rightStep_f, m_swinging_step_source_f, m_swinging_step_f, m_fc_transform);

    if (m_support_step_f->foot == LEFT_FOOT) {
        updateOdometry(leftLeg.getOdoUpdate());
    } else {
        updateOdometry(rightLeg.getOdoUpdate());
    }

    //HACK check to see if we are done - still too soon, but works! (see graphs)
    if (supportStep_s->type == END_STEP && swingingStep_s->type == END_STEP
        && lastStep_s->type == END_STEP && m_x == 0.0 && m_y == 0.0 && m_theta == 0.0) {
        m_done = true;
    }

    return WalkLegsTuple(left, right);
}

/**
 * This method handles updating all the necessary coordinate frames and steps
 * when the support feet change
 */
void StepGenerator::swapSupportLegs() {
    if (currentZMPDSteps.size() + futureSteps.size() <
        MIN_NUM_ENQUEUED_STEPS)
        throw "Insufficient steps";

    //there are at least three elements in the list, pop the obsolete one
    //(currently use last step to determine when to stop, hackish-ish)
    //and the first step is the support one now, the second the swing
    lastStep_s = *currentZMPDSteps.begin();
    currentZMPDSteps.pop_front();
    swingingStep_s = *(++currentZMPDSteps.begin());
    supportStep_s = *currentZMPDSteps.begin();

    supportFoot = (supportStep_s->foot == LEFT_FOOT ?
                   LEFT_SUPPORT : RIGHT_SUPPORT);

    //update the translation matrix between i and f coord. frames
    ufmatrix3 stepTransform = get_fprime_f(supportStep_s);
    m_if_transform = prod(stepTransform, m_if_transform);

    //Express the  destination  and source for the supporting foot and
    //swinging foots locations in f coord. Since the supporting foot doesn't
    //move, we ignore its source.

    //First, do the support foot, which is always at the origin
    const ufvector3 origin = CoordFrame3D::vector3D(0, 0);
    const ufvector3 supp_pos_f = origin;

    //Second, do the source of the swinging leg, which can be calculated
    //using the stepTransform matrix from above
    ufvector3 swing_src_f = prod(stepTransform, origin);

    //Third, do the dest. of the swinging leg, which is more complicated
    //We get the translation matrix that takes points in next f-type
    //coordinate frame, namely the one that will be centered at the swinging
    //foot's destination, and puts them into the current f coord. frame
    const ufmatrix3 swing_reverse_trans =
            get_f_fprime(swingingStep_s);
    //This gives us the position of the swinging foot's destination
    //in the current f frame
    const ufvector3 swing_pos_f = prod(swing_reverse_trans,
                                       origin);

    //finally, we need to know how much turning there will be. Turns out,
    //we can simply read this out of the aforementioned translation matr.
    //this only works because its a 3D homog. coord matr - 4D would break
    double swing_dest_angle = -safe_asin(swing_reverse_trans(1, 0));

    //we use the swinging source to calc. a path for the swinging foot
    //it is not clear now if we will need to angle offset or what
    double swing_src_angle = -safe_asin(stepTransform(1, 0));

    //in the F coordinate frames, we express Steps representing
    // the three footholds from above
    m_support_step_f =
            shared_ptr<Step>(new Step(supp_pos_f(0), supp_pos_f(1),
                                      0.0, *supportStep_s));
    m_swinging_step_f =
            shared_ptr<Step>(new Step(swing_pos_f(0), swing_pos_f(1),
                                      swing_dest_angle, *swingingStep_s));
    m_swinging_step_source_f =
            shared_ptr<Step>(new Step(swing_src_f(0), swing_src_f(1),
                                      swing_src_angle, *lastStep_s));

}

/**
 *  This method fills the ZMP queue with extra zmp based on a step
 *  Note the bug that currently exists with this process (in the timing)
 *  See the header and the README.tex/pdf
 *
 *  There are two types of ZMP patterns for when a step is supporting
 *    - Regular, where there is another step coming after
 *    - End, where the ZMP should move directly under the robot (origin of S)
 */
void StepGenerator::fillZMP(const shared_ptr<Step> newSupportStep) {

    switch (newSupportStep->type) {
        case REGULAR_STEP:
            fillZMPRegular(newSupportStep);
            break;
        case END_STEP: //END and NULL might be the same thing....?
            fillZMPEnd(newSupportStep);
            break;
        default:
            throw "Unsupported Step type";
    }
    newSupportStep->zmpd = true;
}

/**
 * Generates the ZMP reference pattern for a normal step
 */
void
StepGenerator::fillZMPRegular(const shared_ptr<Step> newSupportStep) {
    //update the lastZMPD Step
    const double sign = (newSupportStep->foot == LEFT_FOOT ? 1.0 : -1.0);
    const double last_sign = -sign;

    //The intent of this constant is to be approximately the length of the foot
    //and corresponds to the distance we would like the ZMP to move along the
    //single-support-foot (say 80mm or so?). Might not want it to be linearly
    //interpolated either - maybe stay at a point a bit and then  move in a line
    //HACK/ HOWEVER - the best value for this constant is about -20 right now
    //This could have two+ reasons:
    //1) The controller is inaccurate in getting ref and actual zmp to line up
    //   (In fact, we know this is the case, from the debug graphs. but, what we
    //    dont know is if this is the definite cause of instability)
    //2) The approximations made in the simple PreviewController are finally
    //   hurting us. This could be fixed with an observer
    // in anycase, we'll leave this at -20 for now. (The effect is that
    // the com path 'pauses' over the support foot, which is quite nice)
    double X_ZMP_FOOT_LENGTH = 0.0;// HACK/TODO make this center foot gait->footLengthX;

    // An additional HACK:
    // When we are turning, we have this problem that the direction in which
    // we turn, the opening step is well balanced but the step which brings the
    // foot back is bad. We need to swing more toward the opening step in
    // order to not fall inward.
    const double HACK_AMOUNT_PER_PI_OF_TURN = newSupportStep->m_zmp_config[WP::TURN_ZMP_OFF];
    const double HACK_AMOUNT_PER_1_OF_LATERAL = newSupportStep->m_zmp_config[WP::STRAFE_ZMP_OFF];

    double adjustment = ((newSupportStep->theta / M_PI_double) * HACK_AMOUNT_PER_PI_OF_TURN);
    adjustment += (newSupportStep->m_y - (sign * HIP_OFFSET_Y)) * HACK_AMOUNT_PER_1_OF_LATERAL;

    //Another HACK (ie. zmp is not perfect)
    //This moves the zmp reference to the outside of the foot
    double Y_ZMP_OFFSET = (newSupportStep->foot == LEFT_FOOT ?
                           newSupportStep->m_zmp_config[WP::L_ZMP_OFF_Y] :
                           newSupportStep->m_zmp_config[WP::R_ZMP_OFF_Y]);

    Y_ZMP_OFFSET += adjustment;

    // When we turn, the ZMP offset needs to be corrected for the rotation of
    // newSupportStep. A picture would be very useful here. Someday...
    double y_zmp_offset_x = -sin(std::abs(newSupportStep->theta)) * Y_ZMP_OFFSET;
    double y_zmp_offset_y = cos(newSupportStep->theta) * Y_ZMP_OFFSET;

    //lets define the key points in the s frame. See diagram in paper
    //to use bezier curves, we would need also directions for each point
    const ufvector3 start_s = m_last_zmp_end_s;
    const ufvector3 end_s =
            CoordFrame3D::vector3D(newSupportStep->m_x + m_gait->stance[WP::BODY_OFF_X] + y_zmp_offset_x,
                                   newSupportStep->m_y + sign * y_zmp_offset_y);
    const ufvector3 mid_s =
            CoordFrame3D::vector3D(newSupportStep->m_x + m_gait->stance[WP::BODY_OFF_X] + y_zmp_offset_x - X_ZMP_FOOT_LENGTH,
                                   newSupportStep->m_y + sign * y_zmp_offset_y);

    const ufvector3 start_i = prod(si_Transform, start_s);
    const ufvector3 mid_i = prod(si_Transform, mid_s);
    const ufvector3 end_i = prod(si_Transform, end_s);

    //Now, we interpolate between the three points. The line between
    //start and mid is double support, and the line between mid and end
    //is double support

    //double support - consists of 3 phases:
    //  1) a static portion at start_i
    //  2) a moving (diagonal) portion between start_i and mid_i
    //  3) a static portion at start_i
    //The time is split between these phases according to
    //the constant gait->dblSupInactivePercentage

    //First, split up the frames:
    const int halfNumDSChops = //DS - DoubleStaticChops
            static_cast<int>(static_cast<double>(newSupportStep->m_double_support_frames) *
                             newSupportStep->m_zmp_config[WP::DBL_SUP_STATIC_P] / 2.0);
    const int numDMChops = //DM - DoubleMovingChops
            newSupportStep->m_double_support_frames - halfNumDSChops * 2;

    //Phase 1) - stay at start_i
    for (int i = 0; i < halfNumDSChops; i++) {
        m_zmp_ref_x.push_back(start_i(0));
        m_zmp_ref_y.push_back(start_i(1));
    }

    //phase 2) - move from start_i to
    for (int i = 0; i < numDMChops; i++) {
        ufvector3 new_i = start_i + (static_cast<double>(i) / static_cast<double>(numDMChops)) * (mid_i - start_i);
        m_zmp_ref_x.push_back(new_i(0));
        m_zmp_ref_y.push_back(new_i(1));
    }

    //phase 3) - stay at mid_i
    for (int i = 0; i < halfNumDSChops; i++) {
        m_zmp_ref_x.push_back(mid_i(0));
        m_zmp_ref_y.push_back(mid_i(1));
    }


    //single support -  we want to stay over the new step
    const int numSChops = newSupportStep->m_single_support_frames;
    for (int i = 0; i < numSChops; i++) {
//    const int numSChops = gait->stepDurationFrames;
//    for(int i = 0; i< gait->stepDurationFrames; i++){

        ufvector3 new_i = mid_i + (static_cast<double>(i) / static_cast<double>(numSChops)) * (end_i - mid_i);

        m_zmp_ref_x.push_back(new_i(0));
        m_zmp_ref_y.push_back(new_i(1));
    }

    //update our reference frame for the next time this method is called
    si_Transform = prod(si_Transform, get_s_sprime(newSupportStep));
    //store the end of the zmp in the next s frame:
    m_last_zmp_end_s = prod(get_sprime_s(newSupportStep), end_s);
}

/**
 * Generates the ZMP reference pattern for a step when it is the support step
 * such that it will be the last step before stopping
 */
void StepGenerator::fillZMPEnd(const shared_ptr<Step> newSupportStep) {
    const ufvector3 end_s = CoordFrame3D::vector3D(m_gait->stance[WP::BODY_OFF_X], 0.0);
    const ufvector3 end_i = prod(si_Transform, end_s);
    //Queue a starting step, where we step, but do nothing with the ZMP
    //so push tons of zero ZMP values
    for (unsigned int i = 0; i < newSupportStep->m_step_duration_frames; i++) {
        m_zmp_ref_x.push_back(end_i(0));
        m_zmp_ref_y.push_back(end_i(1));
    }

    //An End step should never move the si_Transform!
    //si_Transform = prod(si_Transform,get_s_sprime(newSupportStep));
    //store the end of the zmp in the next s frame:
    m_last_zmp_end_s = prod(get_sprime_s(newSupportStep), end_s);
}

/**
 * Set the speed of the walk eninge in mm/s and rad/s
 */
void StepGenerator::setSpeed(const double _x, const double _y,
                             const double _theta) {
    //Regardless, we are changing the walk vector, so we need to scrap any future plans
    clearFutureSteps();

    m_x = _x;
    m_y = _y;
    m_theta = _theta;

    if (m_done) {
        //we are starting fresh from a stopped state, so we need to clear all remaining
        //steps and zmp values.
        resetQueues();
        //then we need to pick which foot to start with
        const bool startLeft = decideStartLeft(m_y, m_theta);
        resetSteps(startLeft);
    }
    m_done = false;
}


/**
 * Method to enqueue a specific number of steps and then stop
 * The input should be given in velocities (mm/s)
 */
void StepGenerator::takeSteps(const double _x, const double _y, const double _theta,
                              const int _numSteps) {
    //Ensure that we are currently stopped -- if not, throw warning
    if (!m_done) {
        cout << "Warning!!! Step Command with (" << _x << "," << _y << "," << _theta
             << ") and with " << _numSteps << " Steps were APPENDED because"
             "StepGenerator is already active!!" << endl;
    } else {
        //we are starting fresh from a stopped state, so we need to clear all remaining
        //steps and zmp values.
        resetQueues();
        //then we need to pick which foot to start with
        const bool startLeft = decideStartLeft(_y, _theta);
        resetSteps(startLeft);
        //Adding this step is necessary because it was not added in the start left right
        generateStep(_x, _y, _theta);
        m_done = false;
    }
    for (int i = 0; i < _numSteps; i++) {
        generateStep(_x, _y, _theta);
    }
    //skip generating the end step, because it will be generated automatically:
    m_x = 0.0;
    m_y = 0.0;
    m_theta = 0.0;
}


/**
 *  Set up the walking engine for starting with a swinging step on the left,
 *  if startLeft is true
 */
void StepGenerator::resetSteps(const bool startLeft) {
    //This is the place where we reset the controller each time the walk starts
    //over again.
    //First we reset the controller back to the neutral position
    m_controller_x->initState(m_gait->stance[WP::BODY_OFF_X], 0.0, m_gait->stance[WP::BODY_OFF_X]);
    m_controller_y->initState(0.0, 0.0, 0.0);
    //Each time we restart, we need to reset the estimated sensor ZMP:
    m_zmp_filter = ZmpEKF();
    m_sensor_angles->reset();

    //Third, we reset the memory of where to generate ZMP from steps back to
    //the origin
    si_Transform = CoordFrame3D::identity3D();
    m_last_zmp_end_s = CoordFrame3D::vector3D(0.0, 0.0);

    Foot dummyFoot = LEFT_FOOT;
    Foot firstSupportFoot = RIGHT_FOOT;
    double supportSign = 1.0;
    if (startLeft) {
        //start off in a double support phase where the right leg swings first
        //HOWEVER, since the first support step is END, there will be no
        //actual swinging - the first actual swing will be 2 steps
        //after the first support step, in this case, causing left to swing first
        leftLeg.startRight();
        rightLeg.startRight();
        leftArm.startRight();
        rightArm.startRight();

        //depending on we are starting, assign the appropriate steps
        dummyFoot = RIGHT_FOOT;
        firstSupportFoot = LEFT_FOOT;
        supportSign = 1.0;
        m_next_step_is_feft = false;

    } else { //startRight
        //start off in a double support phase where the left leg swings first
        //HOWEVER, since the first support step is END, there will be no
        //actual swinging - the first actual swing will be 2 steps
        //after the first support step, in this case, causing right to swing first
        leftLeg.startLeft();
        rightLeg.startLeft();
        leftArm.startLeft();
        rightArm.startLeft();

        //depending on we are starting, assign the appropriate steps
        dummyFoot = LEFT_FOOT;
        firstSupportFoot = RIGHT_FOOT;
        supportSign = -1.0;
        m_next_step_is_feft = true;
    }

    //we need to re-initialize the if_Transform matrix to reflect which
    //side the we are starting.
    const ufmatrix3 initStart = CoordFrame3D::translation3D(0.0, supportSign * (HIP_OFFSET_Y));
    m_if_transform.assign(initStart);


    //When we start out again, we need to let odometry know to store
    //the distance covered so far. This needs to happen before
    //we reset any coordinate frames
    resetOdometry(m_gait->stance[WP::BODY_OFF_X], -supportSign * HIP_OFFSET_Y);

    //Support step is END Type, but the first swing step, generated
    //in generateStep, is REGULAR type.
    shared_ptr<Step> firstSupportStep =
            shared_ptr<Step>(new Step(ZERO_WALKVECTOR, *m_gait, firstSupportFoot, ZERO_WALKVECTOR, END_STEP));
    shared_ptr<Step> dummyStep =
            shared_ptr<Step>(new Step(ZERO_WALKVECTOR, *m_gait, dummyFoot));
    //need to indicate what the current support foot is:
    currentZMPDSteps.push_back(dummyStep);//right gets popped right away
    fillZMP(firstSupportStep);
    //addStartZMP(firstSupportStep);
    currentZMPDSteps.push_back(firstSupportStep);//left will be sup. during 0.0 zmp
    lastQueuedStep = firstSupportStep;
}


/**
 *  Creates a new step at the specified location (x,y,theta) specified in mm
 */
void StepGenerator::generateStep(double _x,
                                 double _y,
                                 double _theta) {
    //We have this problem that we can't simply start and stop the robot:
    //depending on the step type, we generate different types of ZMP
    //which means after any given step, only certain other steps types are
    //possible. For example, an END_STEP places the ZMP at 0,0, so it is a bad
    //idea to try to have a REGUALR_STEP follow it.  Also, the START_STEP
    //generates the same ZMP pattern as the REGULAR_STEP, but walking leg
    //wont lift up the leg.
    //PROBLEM:
    // In order to follow these guidlines, we introduce some hackish, error
    //prone code (see below) which must be repeated for all three 'directions'.

    //MUSINGS on better design:
    //1)We should probably have two different StepTypes
    //  one that determines what kind of ZMP we want, and another that
    //  determines if we should lift the foot as we approach the destination
    //  determined by that step.
    //2)We would ideally be able to call Generate step no matter what - i.e.
    //  also to create the starting steps, etc. This probably means we need to
    //  figure out how we are externally starting and stopping this module
    //  and also means we need to store class level state information (another
    //  FSA?)
    //3)All this is contingent on building in the idea that with motion vector=
    //  (0,0,0) we imply that the robot stops.  We could encode this as two
    //  different overall behaviors: check if we want to start, else if we
    //  want to start moving, else if are already moving.
    StepType type;

    if (m_gait->step[WP::WALKING] == WP::NON_WALKING_GAIT) {
        type = END_STEP;
        _x = 0.0;
        _y = 0.0;
        _theta = 0.0;

    } else if (_x == 0 && _y == 0 && _theta == 0) {//stopping, or stopped
//         if(lastQueuedStep->x != 0 || lastQueuedStep->theta != 0 ||
//            (lastQueuedStep->y - (lastQueuedStep->foot == LEFT_FOOT ?
//                                  1:-1)*HIP_OFFSET_Y) != 0)
//             type = REGULAR_STEP;
//         else
        type = END_STEP;

    } else {
        //we are moving somewhere, and we must ensure that the last step
        //we enqued was not an END STEP
        if (lastQueuedStep->type == END_STEP) {
            if (lastQueuedStep->zmpd) {//too late to change it! make this a start
                type = REGULAR_STEP;
                _x = 0.0;
                _y = 0.0;
                _theta = 0.0;
            } else {
                type = REGULAR_STEP;
                lastQueuedStep->type = REGULAR_STEP;
            }
        } else {
            //the last step was either start or reg, so we're fine
            type = REGULAR_STEP;
        }
    }

    //The input here is in velocities. We need to convert it to distances perstep
    //Also, we need to scale for the fact that we can only turn or strafe every other step
    const WalkVector new_walk = {_x, _y, _theta};

    shared_ptr<Step> step(new Step(new_walk, *m_gait, (m_next_step_is_feft ? LEFT_FOOT : RIGHT_FOOT),
                                   lastQueuedStep->m_walk_vector, type));

    futureSteps.push_back(step);
    lastQueuedStep = step;
    //switch feet after each step is generated
    m_next_step_is_feft = !m_next_step_is_feft;
}

/**
 * Method to return the default stance of the robot (including arms)
 *
 */
boost::shared_ptr<vector<double> > StepGenerator::getDefaultStance(const Gait& wp) {
    const ufvector3 lleg_goal = CoordFrame3D::vector3D(-wp.stance[WP::BODY_OFF_X],
                                                       wp.stance[WP::LEG_SEPARATION_Y] * 0.5,
                                                       -wp.stance[WP::BODY_HEIGHT]);
    const ufvector3 rleg_goal = CoordFrame3D::vector3D(-wp.stance[WP::BODY_OFF_X],
                                                       -wp.stance[WP::LEG_SEPARATION_Y] * 0.5,
                                                       -wp.stance[WP::BODY_HEIGHT]);

    const vector<double> lleg = WalkingLeg::getAnglesFromGoal(LLEG_CHAIN, lleg_goal, wp.stance);
    const vector<double> rleg = WalkingLeg::getAnglesFromGoal(RLEG_CHAIN, rleg_goal, wp.stance);

    const vector<double> larm(LARM_WALK_ANGLES, &LARM_WALK_ANGLES[ARM_JOINTS]);
    const vector<double> rarm(RARM_WALK_ANGLES, &RARM_WALK_ANGLES[ARM_JOINTS]);

    boost::shared_ptr<vector<double> > allJoints = boost::make_shared<vector<double> >();

    //now combine all the vectors together
    allJoints->insert(allJoints->end(), larm.begin(), larm.end());
    allJoints->insert(allJoints->end(), lleg.begin(), lleg.end());
    allJoints->insert(allJoints->end(), rleg.begin(), rleg.end());
    allJoints->insert(allJoints->end(), rarm.begin(), rarm.end());
    return allJoints;
}

/**
 * Method returns the transformation matrix that goes between the previous
 * foot ('f') coordinate frame and the next f coordinate frame rooted at 'step'
 */
const ufmatrix3 StepGenerator::get_fprime_f(const shared_ptr<Step> step) {
    const double leg_sign = (step->foot == LEFT_FOOT ? 1.0 : -1.0);

    const double x = step->m_x;
    const double y = step->m_y;
    const double theta = step->theta;

    ufmatrix3 trans_fprime_s =
            CoordFrame3D::translation3D(0, -leg_sign * HIP_OFFSET_Y);

    ufmatrix3 trans_s_f =
            prod(CoordFrame3D::rotation3D(CoordFrame3D::Z_AXIS, -theta),
                 CoordFrame3D::translation3D(-x, -y));
    return prod(trans_s_f, trans_fprime_s);
}

/**
 * DIFFERENT Method, returns the transformation matrix that goes between the f
 * coordinate frame rooted at 'step' and the previous foot ('f') coordinate
 * frame rooted at the last step.  Really just the inverse of the matrix
 * returned by the 'get_fprime_f'
 */
const ufmatrix3 StepGenerator::get_f_fprime(const shared_ptr<Step> step) {
    const double leg_sign = (step->foot == LEFT_FOOT ? 1.0 : -1.0);

    const double x = step->m_x;
    const double y = step->m_y;
    const double theta = step->theta;

    ufmatrix3 trans_fprime_s =
            CoordFrame3D::translation3D(0, leg_sign * HIP_OFFSET_Y);

    ufmatrix3 trans_s_f =
            prod(CoordFrame3D::translation3D(x, y),
                 CoordFrame3D::rotation3D(CoordFrame3D::Z_AXIS, theta));
    return prod(trans_fprime_s, trans_s_f);
}

/**
 * Translates points in the sprime frame into the s frame, where
 * the difference between sprime and s is based on 'step'
 */
const ufmatrix3 StepGenerator::get_sprime_s(const shared_ptr<Step> step) {
    const double leg_sign = (step->foot == LEFT_FOOT ? 1.0 : -1.0);

    const double x = step->m_x;
    const double y = step->m_y;
    const double theta = step->theta;

    const ufmatrix3 trans_f_s =
            CoordFrame3D::translation3D(0, leg_sign * HIP_OFFSET_Y);

    const ufmatrix3 trans_sprime_f =
            prod(CoordFrame3D::rotation3D(CoordFrame3D::Z_AXIS, -theta),
                 CoordFrame3D::translation3D(-x, -y));
    return prod(trans_f_s, trans_sprime_f);
}

/**
 * Yet another DIFFERENT Method, returning the matrix that translates points
 * in the next s frame back to the previous one, based on the intervening
 * Step (s' being the last s frame).
 */
const ufmatrix3 StepGenerator::get_s_sprime(const shared_ptr<Step> step) {
    const double leg_sign = (step->foot == LEFT_FOOT ? 1.0 : -1.0);

    const double x = step->m_x;
    const double y = step->m_y;
    const double theta = step->theta;

    const ufmatrix3 trans_f_s =
            CoordFrame3D::translation3D(0, -leg_sign * HIP_OFFSET_Y);

    const ufmatrix3 trans_sprime_f =
            prod(CoordFrame3D::translation3D(x, y),
                 CoordFrame3D::rotation3D(CoordFrame3D::Z_AXIS, theta));
    return prod(trans_sprime_f, trans_f_s);
}

/**
 * Reset (remove all elements) in both the step queues (even ones which
 * we have already committed to), as well as any zmp reference points
 */
void StepGenerator::resetQueues() {
    futureSteps.clear();
    currentZMPDSteps.clear();
    m_zmp_ref_x.clear();
    m_zmp_ref_y.clear();
}

/**
 * Returns the cumulative odometry changes since the last call
 *
 * The basic idea is to keep track of where the start position is located
 * in two c-type frames. The new c frame is the c frame of the robot at the time
 * this method is called. The old c frame was the c frame of the robot when this
 * method was last called (or alternately since instantiation).
 *
 * The odometry update is then calculated by looking at the difference
 * between the location of the global origin (origin_i) in each of those frames.
 * This allows us to see how to translate, then rotate, from the old c frame
 * to the new one.
 *
 * Note that since we reset the location of the controller when we restart walking
 * it is vital to call 'resetOdometry()' in order to make sure any movement
 * since the last call to getOdometryUpdate doesnt get lost
 */
vector<double> StepGenerator::getOdometryUpdate() {
    const double rotation = -safe_asin(cc_Transform(1, 0));
    const ufvector3 odo = prod(cc_Transform, CoordFrame3D::vector3D(0.0, 0.0));
    const double odoArray[3] = {odo(0), odo(1), rotation};
    //printf("Odometry update is (%g,%g,%g)\n",odoArray[0],odoArray[1],odoArray[2]);
    cc_Transform = CoordFrame3D::translation3D(0.0, 0.0);
    return vector<double>(odoArray, &odoArray[3]);
}

/**
 * Method to reset our odometry counters when coordinate frames are switched
 * Ensures we don't loose odometry information if the walk is restarted.
 */
void StepGenerator::resetOdometry(const double initX, const double initY) {
    cc_Transform = CoordFrame3D::translation3D(-initX, -initY);
}

/**
 * Called once per motion frame to update the odometry
 *
 *  We may not correctly account for the rotation around the S frame
 *  rather than the C frame, which is what we are actually returning.
 */

void StepGenerator::updateOdometry(const vector<double>& deltaOdo) {
    const ufmatrix3 odoUpdate = prod(CoordFrame3D::translation3D(deltaOdo[0],
                                                                 deltaOdo[1]),
                                     CoordFrame3D::rotation3D(CoordFrame3D::Z_AXIS,
                                                              -deltaOdo[2]));
    const ufmatrix3 new_cc_Transform = prod(cc_Transform, odoUpdate);
    cc_Transform = new_cc_Transform;

}

/**
 * Method to figure out when to start swinging with the left vs. right left
 */
const bool StepGenerator::decideStartLeft(const double lateralVelocity,
                                          const double radialVelocity) {
    //Currently, the logic is very simple: if the strafe direction
    //or the turn direction go left, then start that way
    //Strafing takes precedence over turning.
    if (lateralVelocity == 0.0) {
        return radialVelocity > 0.0;
    }
    return lateralVelocity > 0.0;
    //An alternate algorithm might compute a test value like
    // lateralVelocity + 0.5*radialVelocity  and decide on that
}

/**
 * Method to return the arm joints during the walk
 */
WalkArmsTuple StepGenerator::tick_arms() {

    return WalkArmsTuple(leftArm.tick(m_support_step_f),
                         rightArm.tick(m_support_step_f));
}

/**
 * Clears any future steps which are far enough in the future that we
 * haven't committed to them yet
 */
void StepGenerator::clearFutureSteps() {
    //first, we need to scrap all future steps:
    futureSteps.clear();
    if (currentZMPDSteps.size() > 0) {
        lastQueuedStep = currentZMPDSteps.back();
        //Then, we need to make sure that the next step we generate will be of the correct type
        //If the last ZMPd step is left, the next one shouldn't be
        m_next_step_is_feft = (lastQueuedStep->foot != LEFT_FOOT);
    }
}
