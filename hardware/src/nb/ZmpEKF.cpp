#include "nb/ZmpEKF.h"
#include "nb/Kinematics.h"
#include "nb/BasicWorldConstants.h"

using namespace Kinematics;

using namespace boost::numeric;

const double ZmpEKF::beta = 0.1;
const double ZmpEKF::gamma = 0.5;
//const double ZmpEKF::variance  = 100.00;

ZmpEKF::ZmpEKF()
        : EKF<ZmpMeasurement, ZmpTimeUpdate, ZMP_NUM_DIMENSIONS, ZMP_NUM_MEASUREMENTS>(beta, gamma) {
    // ones on the diagonal
    A_k(0, 0) = 1.0;
    A_k(1, 1) = 1.0;

    // Set default values for sensor zmp
    xhat_k(0) = 0.0;
    xhat_k(1) = 0.0;

    //Set uncertainties
    P_k(0, 0) = HIP_OFFSET_Y;
    P_k(1, 1) = HIP_OFFSET_Y;

}

ZmpEKF::~ZmpEKF() {

}


/**
 * Method that gets called when we want to advance the filter to the next state
 */
void ZmpEKF::update(const ZmpTimeUpdate tUp,
                    const ZmpMeasurement zMeasure) {
    timeUpdate(tUp);

    std::vector <ZmpMeasurement> z(1, zMeasure);
    correctionStep(z);
    //noCorrectionStep();
}

EKF<ZmpMeasurement, ZmpTimeUpdate, ZMP_NUM_DIMENSIONS, ZMP_NUM_MEASUREMENTS>::StateVector
ZmpEKF::associateTimeUpdate(ZmpTimeUpdate u_k) {
    static ZmpTimeUpdate lastUpdate = {0.0, 0.0};

    StateVector delta(ZMP_NUM_DIMENSIONS);
    delta(0) = u_k.cur_zmp_x - xhat_k(0);
    delta(1) = u_k.cur_zmp_y - xhat_k(1);

    return delta;
}


const double getDontTrustVariance(const double divergence) {
    //For observations from untrustworth sensors, we inflate the variance
    //very high
    static const double minVariance = 20000.0;
    static const double varianceScale = 4000.0;
    return minVariance + std::abs(divergence) * varianceScale;
}

const double getVariance(const double divergence) {
    //For observations from mostly trustworthy sensors,
    // can use a lower variance
    static const double minVariance = .5;
    static const double varianceScale = 1.5;
    return minVariance + std::abs(divergence) * varianceScale;
}


void ZmpEKF::incorporateMeasurement(ZmpMeasurement z,
                                    StateMeasurementMatrix& H_k,
                                    MeasurementMatrix& R_k,
                                    MeasurementVector& V_k) {
    static const double com_height = 310; //TODO: Move this
    static MeasurementVector last_measurement(
            ublas::scalar_vector<double>(measurementSize, 0.0));

    MeasurementVector z_x(measurementSize);
    z_x(0) = z.comX + com_height / GRAVITY_mss * z.accX;
    z_x(1) = z.comY + com_height / GRAVITY_mss * z.accY;

    // The Jacobian is the identity because the observation space is the same
    // as the state space.
    H_k(0, 0) = 1.0;
    H_k(1, 1) = 1.0;

    V_k = z_x - xhat_k; // divergence


    //MeasurementVector deltaS = z_x - last_measurement;

    R_k(0, 0) = getVariance(V_k(0));//variance;
    R_k(1, 1) = getVariance(V_k(1));//variance;

    last_measurement = z_x;
}
