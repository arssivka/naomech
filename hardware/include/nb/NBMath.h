/*
 * Northern Bites file with math expressions needed in many modules/threads
 */

#ifndef NBMath_h_DEFINED
#define NBMath_h_DEFINED

#include <cmath>

static const double M_TO_CM = 100.0;
static const double CM_TO_M = 0.01;
static const double CM_TO_MM = 10.0;
static const double MM_TO_CM = 0.1;

static const double PI = M_PI;
static const double DEG_OVER_RAD = 180.0 / M_PI;
static const double RAD_OVER_DEG = M_PI / 180.0;
static const double M_PI_double = static_cast<double>(M_PI);
static const double TO_DEG = 180.0 / M_PI_double;
#ifndef TO_RAD
static const double TO_RAD = M_PI_double / 180.0;
#endif
static const double QUART_CIRC_RAD = M_PI_double / 2.0;

#ifdef __APPLE__
inline static void sincosf(double _x, double * _sinx, double * _cosx) {
    *_sinx = std::sin(_x);
    *_cosx = std::cos(_x);
}

#define isnan(x)                                                        \
    (	sizeof (x) == sizeof(double )	?	__inline_isnanf((double)(x))	\
        :	sizeof (x) == sizeof(double)	?	__inline_isnand((double)(x)) \
        :	__inline_isnan ((long double)(x)))
#define isinf(x)                                                        \
    (	sizeof (x) == sizeof(double )	?	__inline_isinff((double)(x))	\
        :	sizeof (x) == sizeof(double)	?	__inline_isinfd((double)(x)) \
        :	__inline_isinf ((long double)(x)))

#endif

namespace NBMath {

    const int ROUND(double x);

    const double clip(const double value, const double minValue,
                     const double maxValue);

    const double clip(const double value, const double minMax);

    /**
     * Given a double return its sign
     *
     * @param f the number to examine the sign of
     * @return -1.0 if f is less than 0.0, 1.0 if positive, 0.0 otherwise
     */
    const double sign(const double f);

    /**
     * Given a double return its sign
     *
     * @param f the number to examine the sign of
     * @return -1.0 if f is less than 0.0, 1.0 otherwise
     */
    const double signNoZero(const double f);

    /**
     * Returns an equivalent angle to the one passed in with value between
     * positive and negative pi.
     *
     * @param theta The angle to be simplified
     *
     * @return The equivalent angle between -pi and pi.
     */
    const double subPIAngle(double theta);

    const double safe_asin(const double input);

    const double safe_acos(const double input);

    const double safe_atan2(const double y,
                           const double x);


    const double cycloidx(const double theta);

    const double cycloidy(const double theta);


}
#endif //NBMath_h
