#ifndef _CONTROLLER_INPUT_H_
#define _CONTROLLER_INPUT_H_

#include <limits>
#include <cmath>

namespace tue
{
namespace control
{

static double INVALID_DOUBLE = std::numeric_limits<double>::quiet_NaN();

inline bool is_set(double v) { return !std::isnan(v); }

struct ControllerInput
{
    ControllerInput()
        : pos_reference(INVALID_DOUBLE), vel_reference(INVALID_DOUBLE),
          acc_reference(INVALID_DOUBLE), measurement(INVALID_DOUBLE) {}

    /// Position reference
    double pos_reference;

    /// Velocity reference
    double vel_reference;

    /// Acceleration reference
    double acc_reference;

    /// measurement
    double measurement;

};

} // end namespace tue

} // end namespace control

#endif
