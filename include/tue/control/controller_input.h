#ifndef _CONTROLLER_INPUT_H_
#define _CONTROLLER_INPUT_H_

#include "tue/control/generic.h"

namespace tue
{
namespace control
{

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
