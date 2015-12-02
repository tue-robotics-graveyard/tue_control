#include "tue/control/setpoint_controller.h"

namespace tue
{

namespace control
{

SetpointController::SetpointController()
{

}

SetpointController::~SetpointController()
{

}

bool SetpointController::configure(tue::Configuration& config, double sample_time)
{
    return true;
}

bool SetpointController::update(double measurement, double reference)
{
    output_ = reference;
    return true;
}

}

}
