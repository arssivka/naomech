
#include "nb/SpringSensor.h"
#include "nb/MotionConstants.h"

using namespace boost::numeric;
using namespace NBMath;
using namespace std;
//#define DEBUG_SPRING

SpringSensor::SpringSensor(const MetaGait * _gait,
    const SensorAxis _axis):
    gait(_gait),
    axis(_axis),
    K_INDEX(axis == X ?
            WP::SPRING_K_X  :  WP::SPRING_K_Y),
    GAMMA_INDEX(axis == X ?
                WP::GAMMA_X  :  WP::GAMMA_Y),
    MAX_INDEX(axis == X ?
                WP::MAX_ANGLE_X:  WP::MAX_ANGLE_Y),
    b(CoordFrame3D::vector3D(0.0,0.0,0.0)),
    c(CoordFrame3D::rowVector3D(1.0,0.0,0.0)),
    name(axis == X ? "springX" : "springY"),
    lastSensorAngle(0.0)
{


    //build statematrix
    A = ublas::identity_matrix<double>(3);
    A(0,1) = A(1,2)= MOTION_FRAME_LENGTH_S;
    A(0,2)= 0.5*std::pow(MOTION_FRAME_LENGTH_S,2);
    A(2,2) = A(1,1) = 0.0;

    //NOTE: might added BETA, a friction coefficient, which damps the velocity:
    //in A(2,1)

    reset();

    updateMatrices();

}

SpringSensor::~SpringSensor()
{}

void SpringSensor::reset(){
//reset state
    x_k = CoordFrame3D::vector3D(0.0,0.0,0.0);
}

void SpringSensor::tick_sensor(const double sensorAngle){
    updateMatrices();

    const double clippedSensorAngle  = NBMath::clip(sensorAngle,
                                                  gait->sensor[MAX_INDEX]);

    //control
    const double u = sensorAngle -x_k(0);
    const ufvector3 x_hat = prod(A,x_k) + b*u;
    x_k = x_hat;

    //finally clip position: reference x_hat to avoid assignment error
    x_k(0) = NBMath::clip(x_hat(0),gait->sensor[MAX_INDEX]);
    //should potentially clip more aggressively things like:
    //when we are near position zero, clip velocity


    lastSensorAngle = sensorAngle;//unused
}

const double SpringSensor::getSensorAngle(){
    return x_k(0);
}


void SpringSensor::updateMatrices(){
    //Update state matrix by selecting the appropriate spring const
    A(2,0) = -gait->sensor[K_INDEX];

    //Update control vector by selecting the appropriate spring const
    b(0) = gait->sensor[GAMMA_INDEX];
}
