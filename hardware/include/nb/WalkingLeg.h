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

#ifndef _WalkingLeg_h_DEFINED
#define _WalkingLeg_h_DEFINED

/**
 *
 * This class implements the basic strucutre of an FSA to model the 
 * state transitions of a robot's leg during walking. Particularly,
 * it switches between single and double support modes.
 *
 * This class is designed to switch states automatically, without external
 * input. The only time it needs external input is when it should start
 * Stopping is handled implicitly by ceasing calls to tick(), followed
 * eventually (potentially?) by a call to startRight/startLeft
 *
 * Take note of the autonomous state switching because it will be critical
 * to make sure the dest_x, dest_y sent to this class are calculated from
 * ZMP values in sync with the state transitions of this class.
 *
 * Currently, the state transitions of this class are based on the walking
 * parameters pointer stored locally. This means that items such as step
 * length, etc are currently not variable per step. This will change eventually
 *
 * Eventually, the tick() method should probably be passed something like a
 * LocalStep, which is a step defined in the c frame. If this step has attr.
 * such as destination for the legs, duration, etc, we should be able to move
 * forward with steps of variable length, etc
 *
 */

#include <cstdio>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include "WalkingConstants.h"
#include "MetaGait.h"
#include "Step.h"
#include "CoordFrame.h"
#include "Kinematics.h"
#include "InverseKinematics.h"
#include "NBMatrixMath.h"
#include "rd/hardware/Robot.h"
#include "nb/SensorAngles.h"


typedef boost::tuple<std::vector<double>,
        std::vector<double> > LegJointStiffTuple;


enum JointStiffIndex {
    JOINT_INDEX,
    STIFF_INDEX
};

class WalkingLeg {
public:
    WalkingLeg(boost::shared_ptr<rd::Robot> robot,
                boost::shared_ptr<SensorAngles> sensor_angles,
               const MetaGait* _gait,
               Kinematics::ChainID id);

    ~WalkingLeg();

    LegJointStiffTuple tick(boost::shared_ptr<Step> step,
                            boost::shared_ptr<Step> swing_src,
                            boost::shared_ptr<Step> _suppoting,
                            NBMath::ufmatrix3 fc_Transform);

    void setSteps(boost::shared_ptr<Step> _swing_src,
                  boost::shared_ptr<Step> _swing_dest,
                  boost::shared_ptr<Step> _suppoting);

    //Hopefully these never need to get called (architecturally).
    //Instead, use methods like startLeft, right etc
    //void setSupportMode(SupportMode newMode){setState(newMode);}
    //void switchSupportMode() {nextState();}

    //methods to setup starting the walk
    void startLeft();

    void startRight();

    //Public FSA methods
    SupportMode getSupportMode() { return state; }

    //True if the next call to tick() will be in a different support mode
    bool isSwitchingSupportMode() { return firstFrame(); }

    bool stateIsDoubleSupport() {
        return state == DOUBLE_SUPPORT ||
               state == PERSISTENT_DOUBLE_SUPPORT;
    };

    bool isSupporting() {
        return state == DOUBLE_SUPPORT ||
               state == PERSISTENT_DOUBLE_SUPPORT || state == SUPPORTING;
    };

    std::vector<double> getOdoUpdate();

    void computeOdoUpdate();

    static std::vector<double>
            getAnglesFromGoal(const Kinematics::ChainID chainID,
                              const NBMath::ufvector3& goal,
                              const double stance[WP::LEN_STANCE_CONFIG]);

private:
    //Execution methods, get called depending on which state the leg is in
    LegJointStiffTuple supporting(NBMath::ufmatrix3 fc_Transform);

    LegJointStiffTuple swinging(NBMath::ufmatrix3 fc_Transform);

    //Consolidated goal handleing
    const std::vector<double> finalizeJoints(const NBMath::ufvector3& legGoal);

    //FSA methods
    void setState(SupportMode newState);

    void switchToNextState();

    SupportMode nextState();

    bool shouldSwitchStates();

    bool firstFrame() { return frameCounter == 0; }

    void assignStateTimes(boost::shared_ptr<Step> step);

//hack
public:
    const double getFootRotation();

private:
    const boost::tuple<const double, const double>
            getAnkleAngles();

    const double getEndStepSensorScale();

    const double getFootRotation_c();

    const double getHipYawPitch();

    void applyHipHacks(double angles[]);

    const std::vector<double> getStiffnesses();

    const boost::tuple<const double, const double> getHipHack(const double HYPAngle);

    inline Kinematics::ChainID getOtherLegChainID();

private:
    std::vector<int> m_keys;

    boost::shared_ptr<rd::Robot> m_robot;
    //FSA Attributes
    SupportMode state;
    SupportMode supportMode; //soon to be deprecated
    unsigned int frameCounter;
    unsigned int doubleSupportFrames;
    unsigned int singleSupportFrames;
    unsigned int cycleFrames;

    //destination attributes
    boost::shared_ptr<Step> cur_dest, swing_src, swing_dest, support_step;
    boost::shared_ptr<SensorAngles> m_sensor_angles;

    //Leg Attributes
    Kinematics::ChainID chainID; //keep track of which leg this is
    const MetaGait* gait;
    double lastJoints[Kinematics::LEG_JOINTS];
    NBMath::ufvector3 goal;
    NBMath::ufvector3 last_goal;
    double lastRotation;
    std::vector<double> odoUpdate;
    int leg_sign; //-1 for right leg, 1 for left leg
    std::string leg_name;

    double m_sensor_angle_x, m_sensor_angle_y;

};

#endif
