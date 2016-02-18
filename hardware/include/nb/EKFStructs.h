#ifndef EKFStructs_h
#define EKFStructs_h
#include <ostream>
//#include "VisualBall.h"

// Structs
// Odometery change
class MotionModel
{
public:
    MotionModel(double f = 0.0, double l = 0.0, double r = 0.0)
        : deltaF(f), deltaL(l), deltaR(r) { }
    MotionModel(const MotionModel& other)
        : deltaF(other.deltaF), deltaL(other.deltaL), deltaR(other.deltaR) { }
    double deltaF;
    double deltaL;
    double deltaR;

    friend std::ostream& operator<< (std::ostream &o, const MotionModel &u) {
        return o << "(" << u.deltaF << ", " << u.deltaL << ", " << u.deltaR
                 << ")";
    }

};


/**
 * BallMeasurement - A non-generic class for holding the basic type information
 *                   required (by Tucker) for running the correctionStep in EKF
 */
/*struct RangeBearingMeasurement
{
    RangeBearingMeasurement(double _dist = 0.0, double _bearing = 0.0,
                            double _distSD = 0.0, double _bearingSD = 0.0) :
        distance(_dist), bearing(_bearing), distanceSD(_distSD),
        bearingSD(_bearingSD) {}
    RangeBearingMeasurement(const RangeBearingMeasurement& other) :
        distance(other.distance), bearing(other.bearing),
        distanceSD(other.distanceSD), bearingSD(other.bearingSD) {}
    RangeBearingMeasurement(VisualBall * ball) :
        distance(ball->getDistance()), bearing(ball->getBearing()),
        distanceSD(ball->getDistanceSD()), bearingSD(ball->getBearingSD()) {}

    friend std::ostream& operator<< (std::ostream &o,
                                     const RangeBearingMeasurement &m) {
        return o << "(" << m.distance << ", " << m.bearing << ", "
                 << m.distanceSD << ", " << m.bearingSD << ")";
    }

    double distance;
    double bearing;
    double distanceSD;
    double bearingSD;
};*/


/**
 * AccelMeasurement - A non-generic class for holding accelerometer values
 *                    required (by motion) for filtering accel sensor values.
 */
struct AccelMeasurement {
    double x;
    double y;
    double z;
};

/**
 * AngleMeasurement - a class for holding angleX, angleY
 */

struct AngleMeasurement{
  double angleX;
  double angleY;
};


/**
 * ZmpTimeUpdate - For ZmpEKF, the apriori input comes as the zmp from the
 *                 third entry in the controller's state vector.
 *
 */

struct ZmpTimeUpdate{
    double cur_zmp_x;
    double cur_zmp_y;
};

/**
 * AccelMeasurement - A non-generic class for holding accelerometer values
 *                    required (by motion) for filtering accel sensor values.
 */
struct ZmpMeasurement {
    double comX;
    double comY;
    double accX;
    double accY;
};

#endif
