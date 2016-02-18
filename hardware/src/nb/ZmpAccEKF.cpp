#include "nb/ZmpAccEKF.h"
#include "nb/BasicWorldConstants.h"

using namespace boost::numeric;

const int ZmpAccEKF::num_dimensions = ACC_NUM_DIMENSIONS;
const double ZmpAccEKF::beta = 0.2;
const double ZmpAccEKF::gamma = .2;
const double ZmpAccEKF::variance = 0.22;
//const double ZmpAccEKF::variance  = 100.00;

ZmpAccEKF::ZmpAccEKF()
        : EKF<AccelMeasurement, int, num_dimensions, num_dimensions>(beta, gamma) {
    // ones on the diagonal
    A_k(0, 0) = 1.0;
    A_k(1, 1) = 1.0;
    A_k(2, 2) = 1.0;

    // Set default values for the accelerometers
    xhat_k(0) = 0.0;
    xhat_k(1) = 0.0;
    xhat_k(2) = GRAVITY_mss;

    //Set uncertainties
    P_k(0, 0) = -GRAVITY_mss;
    P_k(1, 1) = -GRAVITY_mss;
    P_k(2, 2) = -GRAVITY_mss;


}

ZmpAccEKF::~ZmpAccEKF() {

}


/**
 * Method that gets called when we want to advance the filter to the next state
 */
void ZmpAccEKF::update(const double accX,
                       const double accY,
                       const double accZ) {
    timeUpdate(0); // update model? we don't have one. it's an int. don't care.

    AccelMeasurement m = {accX, accY, accZ};
    // we create a vector of size 1 and default value m
    std::vector <AccelMeasurement> z(1, m);
    correctionStep(z);
}

EKF<AccelMeasurement, int, 3, 3>::StateVector
ZmpAccEKF::associateTimeUpdate(int u_k) {
    return ublas::zero_vector<double>(num_dimensions);
}

const double ZmpAccEKF::scale(const double x) {
    //return .4 * std::pow(3.46572, x);
    // Highly filtered: return 100.0 * std::pow(x, 5.0) + 580.4;
    // A bezier curve - pretty tight fit, still lags a bit
//     return 6.73684 * std::pow(x,3) +
//        37.8947 * std::pow(x,2) +
//        -54.6316 * x +
//        20.0;

    //very tight fit
    return 2.0 * std::pow(3.0, x);

    //Looser fit:
//     return 6.73684 * std::pow(x,3) +
//        37.8947 * std::pow(x,2) +
//        -54.6316 * x +
//        70.0;

    //return 80 - 79 * std::exp( - .36 * std::pow( - 2.5 + x , 2));

//     if (x > 9.0)
//         return 400.0;
//     else
//         return 80 - 79 * std::exp( - .25 * std::pow( - 2.7 + x , 2));
}

const double ZmpAccEKF::getVariance(double delta, double divergence) {
    delta = std::abs(delta);
    divergence = std::abs(divergence);

    const double big = 3.5;
    const double small = 1.0;
    const double trust = .2;
    const double dont_trust = 1000.0;

    if (delta > big && divergence < small) {
        return trust;
    }

    if (delta < small && divergence < small) {
        return trust;
    }

    return dont_trust;
}

void ZmpAccEKF::incorporateMeasurement(AccelMeasurement z,
                                       StateMeasurementMatrix& H_k,
                                       MeasurementMatrix& R_k,
                                       MeasurementVector& V_k) {
    static MeasurementVector last_measurement(
            ublas::scalar_vector<double>(num_dimensions, 0.0));

    MeasurementVector z_x(num_dimensions);
    z_x(0) = z.x;
    z_x(1) = z.y;
    z_x(2) = z.z; // hahahha

    V_k = z_x - xhat_k; // divergence

    // The Jacobian is the identity because the observation space is the same
    // as the state space.
    H_k(0, 0) = 1.0;
    H_k(1, 1) = 1.0;
    H_k(2, 2) = 1.0;

    //
    MeasurementVector deltaS = z_x - last_measurement;

/*
    R_k(0,0) = getVariance(deltaS(0), V_k(0));
    R_k(1,1) = getVariance(deltaS(1), V_k(1));
    R_k(2,2) = getVariance(deltaS(2), V_k(2));

*/

    // Update the measurement covariance matrix
    R_k(0, 0) = scale(std::abs(deltaS(0)));
    R_k(1, 1) = scale(std::abs(deltaS(1)));
    R_k(2, 2) = scale(std::abs(deltaS(2)));

//     R_k(0,0) = scale(std::abs(V_k(0)));
//     R_k(1,1) = scale(std::abs(V_k(1)));
//     R_k(2,2) = scale(std::abs(V_k(2)));
    last_measurement = z_x;
}
