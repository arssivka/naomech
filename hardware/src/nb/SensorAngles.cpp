#include "nb/SensorAngles.h"
#include "nb/MotionConstants.h"
#include "nb/NBMath.h"

using boost::shared_ptr;

//NOTE/DANGER gait constants which work well with one model may
//suck horibly with the other mode!!
#define USE_SPRING

SensorAngles::SensorAngles(shared_ptr<rd::Robot> robot,
    const MetaGait * _gait):
    m_robot(robot),
    gait(_gait),
    sensorAngleX(0.0),sensorAngleY(0.0),
    springX(gait,SpringSensor::X),
    springY(gait,SpringSensor::Y),
    lastSensorAngleX(0.0),lastSensorAngleY(0.0)
{}


SensorAngles::~SensorAngles(){}


void SensorAngles::tick_sensors(){
    if(gait->sensor[WP::FEEDBACK_TYPE] == 1.0)
        spring_sensor_feedback();
    else if(gait->sensor[WP::FEEDBACK_TYPE] == 0.0)
        basic_sensor_feedback();
    else{
        spring_sensor_feedback();
        // std::cout << gait->sensor[WP::FEEDBACK_TYPE]
        //           <<" is not a valid sensor feedback type"<<std::endl;
    }
}


void SensorAngles::spring_sensor_feedback(){
    //const Inertial inertial = sensors->getInertial();
    //std::cout << "AngleX/Y  = ("<<inertial.angleX<<","<<inertial.angleY<<")"<<std::endl;
    const rd::SensorData<double>::Ptr& angle = m_robot->getAngle()->getAngle();
    springX.tick_sensor(angle->data[rd::Angle::X]);
    springY.tick_sensor(angle->data[rd::Angle::Y] - gait->stance[WP::BODY_ROT_Y]);

    sensorAngleX = springX.getSensorAngle();
    sensorAngleY = springY.getSensorAngle();
}

void SensorAngles::basic_sensor_feedback(){
    const double MAX_SENSOR_ANGLE_X = gait->sensor[WP::MAX_ANGLE_X];
    const double MAX_SENSOR_ANGLE_Y = gait->sensor[WP::MAX_ANGLE_Y];

    const double MAX_SENSOR_VEL = gait->sensor[WP::MAX_ANGLE_VEL]*
        MOTION_FRAME_LENGTH_S;

    //calculate the new angles, take into account gait angles already
    //Inertial inertial = sensors->getInertial();

    const rd::SensorData<double>::Ptr& angle = m_robot->getAngle()->getAngle();
    const double desiredSensorAngleX = angle->data[0] * gait->sensor[WP::GAMMA_X];
    const double desiredSensorAngleY = (angle->data[1] - gait->stance[WP::BODY_ROT_Y])
        *gait->sensor[WP::GAMMA_X];

    //Clip the velocities, and max. limits
    sensorAngleX =
        NBMath::clip(
            NBMath::clip(desiredSensorAngleX,
                         desiredSensorAngleX - MAX_SENSOR_VEL,
                         desiredSensorAngleX + MAX_SENSOR_VEL),
            MAX_SENSOR_ANGLE_X);
    sensorAngleY =
        NBMath::clip(
            NBMath::clip(desiredSensorAngleY,
                         desiredSensorAngleY - MAX_SENSOR_VEL,
                         desiredSensorAngleY + MAX_SENSOR_VEL),
            MAX_SENSOR_ANGLE_Y);

    lastSensorAngleX = sensorAngleX;
    lastSensorAngleY = sensorAngleY;


}
void SensorAngles::reset(){
    lastSensorAngleX = lastSensorAngleY = 0.0;
    sensorAngleX=  sensorAngleY = 0.0;
}


/*
 * Get the sensor based adjustment to the body's rotation
 *
 */
const boost::tuple<const double, const double>
SensorAngles::getAngles(const double scale) const {

    return boost::tuple<const double, const double> (sensorAngleX*scale,
                                                   sensorAngleY*scale);
}
