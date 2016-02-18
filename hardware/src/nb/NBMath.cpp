#include "nb/NBMath.h"

using namespace NBMath;

const int NBMath::ROUND(double x) {
    if ((x - static_cast<double>(
            static_cast<int>(x))) >= 0.5)
        return (static_cast<int>(x) + 1);
    if ((x - static_cast<double>(
            static_cast<int>(x))) <= -0.5)
        return (static_cast<int>(x) - 1);
    else return (int) x;
}

const double NBMath::clip(const double value, const double minValue,
                         const double maxValue) {
    if (value > maxValue)
        return maxValue;
    else if (value < minValue)
        return minValue;
    else if (isnan(value))
        return 0.0;
    else
        return value;
}

const double NBMath::clip(const double value, const double minMax) {
    return clip(value, -minMax, minMax);
}

/**
 * Given a double return its sign
 *
 * @param f the number to examine the sign of
 * @return -1.0 if f is less than 0.0, 1.0 if greater than, 0.0 otherwise
 */
const double NBMath::sign(const double f) {
    if (f < 0.0) {
        return -1.0;
    } else if (f > 0.0) {
        return 1.0;
    } else {
        return 0.0;
    }
}

/**
 * Given a double return its sign
 *
 * @param f the number to examine the sign of
 * @return -1.0 if f is less than 0.0, 1.0 otherwise
 */
const double NBMath::signNoZero(const double f) {
    if (f < 0.0) {
        return -1.0;
    } else {
        return 1.0;
    }
}

/**
 * Returns an equivalent angle to the one passed in with value between positive
 * and negative pi.
 *
 * @param theta The angle to be simplified
 *
 * @return The equivalent angle between -pi and pi.
 */
const double NBMath::subPIAngle(double theta) {
    theta = std::fmod(theta, 2.0 * M_PI_double);
    if (theta > M_PI) {
        theta -= 2.0 * M_PI_double;
    }

    if (theta < -M_PI) {
        theta += 2.0 * M_PI_double;
    }
    return theta;
}

const double NBMath::safe_asin(const double input) {
    return std::asin(clip(input, 1.0));
}

const double NBMath::safe_acos(const double input) {
    return std::acos(clip(input, 1.0));
}


const double  NBMath::cycloidx(const double theta) {
    return theta - std::sin(theta);
}

const double  NBMath::cycloidy(const double theta) {
    return 1.0 - std::cos(theta);
}

const double NBMath::safe_atan2(const double y, const double x) {
    if (x == 0.0) {
        if (y > 0.0) {
            return M_PI_double / 2.0;
        } else {
            return -M_PI_double / 2.0;
        }
    }
    return atan2(y, x);
}
