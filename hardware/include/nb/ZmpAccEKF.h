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
#ifndef _ZmpAccEKF_h_DEFINED
#define _ZmpAccEKF_h_DEFINED

/**
 * Class for filtering accelerometer values received from sensors. Extends the
 * abstract EKF class.
 */

#include "EKF.h"
#include "EKFStructs.h"

class ZmpAccEKF : public EKF<AccelMeasurement, int, ACC_NUM_DIMENSIONS,
        ACC_NUM_DIMENSIONS> {
public:
    ZmpAccEKF();

    virtual ~ZmpAccEKF();

    // Update functions
    void update(const double accx,
                const double accy,
                const double accz);

    // getters
    const double getX() const { return xhat_k(0); }

    const double getY() const { return xhat_k(1); }

    const double getZ() const { return xhat_k(2); }

    const double getXUnc() const { return P_k(0, 0); }

    const double getYUnc() const { return P_k(1, 1); }

    const double getZUnc() const { return P_k(2, 2); }

private:
    // Core functions
    virtual StateVector associateTimeUpdate(int u_k);

    virtual void incorporateMeasurement(AccelMeasurement z,
                                        StateMeasurementMatrix& H_k,
                                        MeasurementMatrix& R_k,
                                        MeasurementVector& V_k);

    const double scale(const double);

    const double getVariance(double, double);

private: // Constants
    static const int num_dimensions;
    static const double beta;
    static const double gamma;
    static const double variance;
};

#endif
