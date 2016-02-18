#include "nb/AbstractGait.h"
#include <iostream>
#include "string.h"
#include "stdio.h"

using namespace std;

AbstractGait::AbstractGait() { }

AbstractGait::~AbstractGait() { }


void AbstractGait::setGaitFromArrays(
        const double _stance[WP::LEN_STANCE_CONFIG],
        const double _step[WP::LEN_STEP_CONFIG],
        const double _zmp[WP::LEN_ZMP_CONFIG],
        const double _hack[WP::LEN_HACK_CONFIG],
        const double _sensor[WP::LEN_SENSOR_CONFIG],
        const double _stiffness[WP::LEN_STIFF_CONFIG],
        const double _odo[WP::LEN_ODO_CONFIG],
        const double _arm[WP::LEN_ARM_CONFIG]) {
    memcpy(stance, _stance, WP::LEN_STANCE_CONFIG * sizeof(double));
    memcpy(step, _step, WP::LEN_STEP_CONFIG * sizeof(double));
    memcpy(zmp, _zmp, WP::LEN_ZMP_CONFIG * sizeof(double));
    memcpy(hack, _hack, WP::LEN_HACK_CONFIG * sizeof(double));

    memcpy(sensor, _sensor, WP::LEN_SENSOR_CONFIG * sizeof(double));
    memcpy(stiffness, _stiffness, WP::LEN_STIFF_CONFIG * sizeof(double));
    memcpy(odo, _odo, WP::LEN_ODO_CONFIG * sizeof(double));
    memcpy(arm, _arm, WP::LEN_ARM_CONFIG * sizeof(double));

}


void AbstractGait::setGaitFromGait(const AbstractGait& other) {
    setGaitFromArrays(other.stance,
                      other.step,
                      other.zmp,
                      other.hack,
                      other.sensor,
                      other.stiffness,
                      other.odo,
                      other.arm);
}

template<const unsigned int length>
void AbstractGait::addSubComponent(double target[length],
                                   const double array1[length],
                                   const double array2[length]) {
    for (unsigned int i = 0; i < length; i++) {
        target[i] = array1[i] + array2[i];
    }
}

template<const unsigned int length>
void AbstractGait::multiplySubComponent(double target[length],
                                        const double source[length],
                                        const double scalar) {
    for (unsigned int i = 0; i < length; i++) {
        target[i] = source[i] * scalar;
    }
}


void AbstractGait::interpolateGaits(AbstractGait& targetGait,
                                    const AbstractGait& startGait,
                                    const AbstractGait& endGait,
                                    const double percentComplete) {

    if (percentComplete == 0.0) {
        targetGait = startGait;
        return;
    }
    if (percentComplete == 1.0) {
        targetGait = endGait;
        return;
    }
    //NOTE attributes which are interpolated COMPLETE  _should_ be
    // also stored as an attribute of the step when it is created
    // from a gait.

    const double COMPLETE = 1.0;
    //For each component of the gait, make a new combination depending
    //on how far in the switching process we are.
    //Some gait components are associated with steps,
    //so they aren't interpolated
    combineSubComponents<WP::LEN_STANCE_CONFIG>
            (targetGait.stance, startGait.stance, endGait.stance, percentComplete);
    combineSubComponents<WP::LEN_STEP_CONFIG>
            (targetGait.step, startGait.step, endGait.step, COMPLETE);
    combineSubComponents<WP::LEN_ZMP_CONFIG>
            (targetGait.zmp, startGait.zmp, endGait.zmp, COMPLETE);
    combineSubComponents<WP::LEN_HACK_CONFIG>
            (targetGait.hack, startGait.hack, endGait.hack, percentComplete);
    combineSubComponents<WP::LEN_SENSOR_CONFIG>
            (targetGait.sensor, startGait.sensor, endGait.sensor, percentComplete);
    combineSubComponents<WP::LEN_STIFF_CONFIG>
            (targetGait.stiffness, startGait.stiffness,
             endGait.stiffness, percentComplete);
    combineSubComponents<WP::LEN_ODO_CONFIG>
            (targetGait.odo, startGait.odo, endGait.odo, percentComplete);
    combineSubComponents<WP::LEN_ARM_CONFIG>
            (targetGait.arm, startGait.arm, endGait.arm, percentComplete);
}


template<const unsigned int length>
void AbstractGait::combineSubComponents(double target[length],
                                        const double source1[length],
                                        const double source2[length],
                                        const double percentSwitched) {
    double temp1[length];
    double temp2[length];


    const double source1Contribution = 1.0 - percentSwitched;
    const double source2Contribution = percentSwitched;

    // cout << " *** New recombination ** sC1,sC2 ="<<source1Contribution
    //      <<","<<source2Contribution<<endl;
    // cout << "Source1 is:"<<endl<<"   [";
    // for(unsigned int  i = 0; i <  length; i++){
    //     cout << source1[i]<<",";
    // }cout<<"]"<<endl;
    // cout << "Source2 is:"<<endl<<"   [";
    // for(unsigned int  i = 0; i <  length; i++){
    //     cout << source2[i]<<",";
    // }cout<<"]"<<endl;

    if (percentSwitched == 0.0) {
        memcpy(target, source1, sizeof(double) * length);
        return;
    }
    if (percentSwitched == 1.0) {
        memcpy(target, source2, sizeof(double) * length);
        return;
    }
    multiplySubComponent<length>(temp1, source1, source1Contribution);
    multiplySubComponent<length>(temp2, source2, source2Contribution);
    addSubComponent<length>(target, temp1, temp2);
    //     cout << "Result is:"<<endl<<"   [";
    // for(unsigned int  i = 0; i <  length; i++){
    //     cout << target[i]<<",";
    // }cout<<"]"<<endl<<endl;
}

using namespace WP;

std::string AbstractGait::toString() const {
    string out;
    char temp[200];

    out += "#### STANCE #####\n";
    for (int i = 0; i < LEN_STIFF_CONFIG; i++) {
        sprintf(temp, "%f,", stance[i]);
        out += string(temp);
    }
    out += "\n#### STEP #####\n";

    for (int i = 0; i < LEN_STEP_CONFIG; i++) {
        sprintf(temp, "%f,", step[i]);
        out += string(temp);
    }
    out += "\n#### ZMP #####\n";

    for (int i = 0; i < LEN_ZMP_CONFIG; i++) {
        sprintf(temp, "%f,", zmp[i]);
        out += string(temp);
    }
    out += "\n#### HACK #####\n";

    for (int i = 0; i < LEN_HACK_CONFIG; i++) {
        sprintf(temp, "%f,", hack[i]);
        out += string(temp);
    }
    out += "\n#### SENSOR #####\n";

    for (int i = 0; i < LEN_SENSOR_CONFIG; i++) {
        sprintf(temp, "%f,", sensor[i]);
        out += string(temp);
    }
    out += "\n#### STIFF #####\n";

    for (int i = 0; i < LEN_STIFF_CONFIG; i++) {
        sprintf(temp, "%f,", stiffness[i]);
        out += string(temp);
    }
    out += "\n#### ODO #####\n";

    for (int i = 0; i < LEN_ODO_CONFIG; i++) {
        sprintf(temp, "%f,", odo[i]);
        out += string(temp);
    }
    out += "\n#### ARM #####\n";

    for (int i = 0; i < LEN_ARM_CONFIG; i++) {
        sprintf(temp, "%f,", arm[i]);
        out += string(temp);
    }
    out += "\n#########\n";
    return out;
}
