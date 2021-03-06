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

/**
 * Class for filtering accelerometer values received from sensors. Extends the
 * abstract EKF class.
 */

#include "nb/EKF.h"
#include "nb/EKFStructs.h"

class ZmpEKF : public EKF<ZmpMeasurement, ZmpTimeUpdate, ZMP_NUM_DIMENSIONS,
        ZMP_NUM_MEASUREMENTS> {
public:
    ZmpEKF();

    virtual ~ZmpEKF();

    // Update functions
    void update(const ZmpTimeUpdate tUp,
                const ZmpMeasurement zMeasure);

    // getters
    const double get_zmp_x() const { return xhat_k(0); }

    const double get_zmp_y() const { return xhat_k(1); }

    const double get_zmp_unc_x() const { return P_k(0, 0); }

    const double get_zmp_unc_y() const { return P_k(1, 1); }

private:
    // Core functions
    virtual StateVector associateTimeUpdate(ZmpTimeUpdate u_k);

    virtual void incorporateMeasurement(ZmpMeasurement z,
                                        StateMeasurementMatrix& H_k,
                                        MeasurementMatrix& R_k,
                                        MeasurementVector& V_k);

private: // Constants
    static const double beta;
    static const double gamma;
};
