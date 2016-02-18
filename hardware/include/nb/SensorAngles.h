#ifndef SensorAngles_h_DEFINED
#define SensorAngles_h_DEFINED

#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include "rd/hardware/Robot.h"
#include "MetaGait.h"
#include "SpringSensor.h"

class SensorAngles{
public:
    SensorAngles(boost::shared_ptr<rd::Robot> robot, const MetaGait * _gait);
    ~SensorAngles();


    void tick_sensors();
    void reset();

    //tuple indices
    enum SensorAxis{
        X = 0,
        Y
    };

    const boost::tuple<const double, const double>
    getAngles(const double scale = 1.0f) const ;

private:
    void basic_sensor_feedback();
    void spring_sensor_feedback();

private:
    boost::shared_ptr<rd::Robot> m_robot;
    const MetaGait * gait;

    //store what will be returned by getAngles
    double sensorAngleX, sensorAngleY;

    SpringSensor springX,springY;

    //OLD
    //State info
    //sensor feedback stuff
    double lastSensorAngleX,lastSensorAngleY;

};


#endif
