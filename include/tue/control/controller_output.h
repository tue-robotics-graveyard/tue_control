#ifndef _CONTROLLER_OUTPUT_H_
#define _CONTROLLER_OUTPUT_H_

#include "tue/control/generic.h"


namespace tue
{
namespace control
{

struct ControllerOutput
{
    ControllerOutput()
        : value(INVALID_DOUBLE), error(INVALID_DOUBLE) {}

    double value;

    double error;

};

} // end namespace tue

} // end namespace control

#endif
