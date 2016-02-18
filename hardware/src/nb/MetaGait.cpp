#include  "nb/MetaGait.h"
#include  "nb/MotionConstants.h"

//using namespace std;

MetaGait::MetaGait() :
        curGait(DEFAULT_GAIT),
        nextGait(DEFAULT_GAIT),
        newGait(DEFAULT_GAIT),
        newGaitSent(false),
        transitionCounter(0),
        transitionFrames(0) { setGaitFromGait(DEFAULT_GAIT); }

MetaGait::~MetaGait() { }

void MetaGait::tick_gait() {
    if (updateGaits()) {
        interpolateGaits(*this, curGait, nextGait, getPercentComplete());
    }
}

void MetaGait::setNewGaitTarget(Gait& nextTarget) {

    newGait = nextTarget;
    newGaitSent = true;
}

void MetaGait::setStartGait(Gait& newCurGait) {
    curGait = newCurGait;
    nextGait = newCurGait;
    resetTransitioning();
}

double MetaGait::getPercentComplete() {
    if (transitionFrames == 0)
        return 1.0;
    return NBMath::clip(static_cast<double>(transitionCounter) /
                        static_cast<double>(transitionFrames),
                        0.0, 1.0);
}

void MetaGait::resetTransitioning() {
    //First, find the maximum
    const double maxTime = std::max(curGait.stance[WP::TRANS_TIME],
                                   nextGait.stance[WP::TRANS_TIME]);
    transitionFrames = static_cast<unsigned int>(maxTime / MOTION_FRAME_LENGTH_S);

    transitionCounter = 1;
}

bool MetaGait::updateGaits() {
    if (newGaitSent) {
        //Make a hybrid gait from the currently selected gaits
        AbstractGait hybridGait;
        const double percComplete = getPercentComplete();
        interpolateGaits(hybridGait, curGait, nextGait, percComplete);

        //Swap the gaits
        curGait = hybridGait;
        nextGait = newGait;

        //Reset the counters
        resetTransitioning();
        newGaitSent = false;
    } else {
        transitionCounter = std::min(transitionCounter + 1, transitionFrames + 1);
    }

    //we still need more processing,
    return transitionCounter <= transitionFrames;
}





