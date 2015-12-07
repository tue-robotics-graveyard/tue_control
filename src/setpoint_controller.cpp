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

void SetpointController::configure(tue::Configuration& config, double sample_time)
{
}

void SetpointController::update(const ControllerInput& input)
{
    if (!is_set(input.pos_reference))
        return;

    output_ = input.pos_reference;
    return;
}

}

}
