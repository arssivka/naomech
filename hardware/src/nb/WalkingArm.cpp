#include "nb/WalkingArm.h"


using namespace Kinematics;
using boost::shared_ptr;
using namespace std;

WalkingArm::WalkingArm(const MetaGait* _gait, ChainID id)
        : state(SUPPORTING),
          chainID(id),
          gait(_gait),
          frameCounter(0),
          startStep(true),
          lastStepType(REGULAR_STEP) { }

WalkingArm::~WalkingArm() { }


ArmJointStiffTuple WalkingArm::tick(shared_ptr <Step> supportStep) {
    singleSupportFrames = supportStep->m_single_support_frames;
    doubleSupportFrames = supportStep->m_double_support_frames;

    vector<double> armJoints = (chainID == LARM_CHAIN ?
                               vector<double>(LARM_WALK_ANGLES,
                                             &LARM_WALK_ANGLES[ARM_JOINTS]) :
                               vector<double>(RARM_WALK_ANGLES,
                                             &RARM_WALK_ANGLES[ARM_JOINTS]));

    armJoints[0] += getShoulderPitchAddition(supportStep);
    vector<double> armStiffnesses(ARM_JOINTS, gait->stiffness[WP::ARM]);
    armStiffnesses[0] = gait->stiffness[WP::ARM_PITCH];

    frameCounter++;
    for (unsigned int i = 0; shouldSwitchStates() && i < 2; i++) {
        switchToNextState();
        lastStepType = supportStep->type;
    };

    return ArmJointStiffTuple(armJoints, armStiffnesses);
}

/*
 * Currently, the arms only move in the forward direction by modulating the
 * shoulderPitch
 */
const double WalkingArm::getShoulderPitchAddition(shared_ptr <Step> supportStep) {
    double direction = 1.0; //forward = negative
    double percentComplete = 0.0;
    switch (state) {
        case SUPPORTING:
            //When the leg on this side is supporting (i.e. swinging back)
            //this arm should swing forward with the foot from the other side
            direction = -1.0;
            percentComplete = static_cast<double>(frameCounter) /
                              static_cast<double>(singleSupportFrames);
            break;
        case SWINGING:
            //When the leg on this side is swinging (i.e. swinging forward)
            //this arm should swing backward with the foot from the other side
            direction = 1.0;
            percentComplete = static_cast<double>(frameCounter) /
                              static_cast<double>(singleSupportFrames);
            break;
        case DOUBLE_SUPPORT:
            //When the leg on this side is in non-persistent double support,
            //it was just recently SUPPORTING, so this arm was swinging back
            //so during double support it should stay back
            direction = -1.0;
            percentComplete = 1.0;
            break;
        case PERSISTENT_DOUBLE_SUPPORT:
            //When the leg on this side is in persistent double support,
            //it was just recently SWINGING, so this arm was swinging forwward
            //so during persistent double support it should stay forward
            direction = 1.0;
            percentComplete = 1.0;
            break;
    }

    double start = -direction * gait->arm[WP::AMPLITUDE];
    double end = direction * gait->arm[WP::AMPLITUDE];

    //We need to intelligently deal with non-regular steps
    //Since end steps are employed in both the starting and stopping contexts
    //and since we don't pay attention to the src, dest, etc for the swinging
    //leg as in WalkingLeg, we need to make sure we do the right thing in each case
    //The manipulations of end, start targets for the pitch are a bit hackish
    //but they work.
    //also note the problem of getting two end steps in a row, since in such a case
    //the arms should be held at their default values
    if (supportStep->type == END_STEP) {
        if (lastStepType == END_STEP) {
            start = end = 0.0;
        } else if (startStep) {
            start = 0.0;
            end = -end;
        } else {
            end = 0.0;
            start = -start;
        }
        //all end steps should be entirely composed of DOUBLE SUPPORT!
        percentComplete = static_cast<double>(frameCounter) /
                          static_cast<double>(doubleSupportFrames);
    }

    //Even though we already calcualted percent complete, we should really
    //have a more gradual arm motion, which we can do by employing a cycloid

    const double theta = percentComplete * 2.0 * M_PI_double;
    const double percentToDest = NBMath::cycloidx(theta) / (2.0 * M_PI_double);

    return start + percentToDest * (end - start);

}

void WalkingArm::startRight() {
    if (chainID == LARM_CHAIN) {
        //we will start walking first by swinging right leg (not this leg), so
        //we want persistent double support
        setState(PERSISTENT_DOUBLE_SUPPORT);
    } else {
        setState(DOUBLE_SUPPORT);
    }
    startStep = true;
    lastStepType = REGULAR_STEP;
}

void WalkingArm::startLeft() {
    if (chainID == LARM_CHAIN) {
        //we will start walking first by swinging left leg (this leg), so
        //we want double, not persistent, support
        setState(DOUBLE_SUPPORT);
    } else {
        setState(PERSISTENT_DOUBLE_SUPPORT);
    }
    startStep = true;
    lastStepType = REGULAR_STEP;
}

bool WalkingArm::shouldSwitchStates() {
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

void WalkingArm::switchToNextState() {
    setState(nextState());
    startStep = false;
}

SupportMode WalkingArm::nextState() {
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

void WalkingArm::setState(SupportMode newState) {
    state = newState;
    frameCounter = 0;
}
