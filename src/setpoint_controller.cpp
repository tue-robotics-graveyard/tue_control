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

void SetpointController::configure(tue::Configuration& config, double dt)
{
}

void SetpointController::update(const ControllerInput& input, ControllerOutput& output)
{
    if (!is_set(input.pos_reference))
        return;

    output.error = input.pos_reference - input.measurement;
    output.value = input.pos_reference;

    return;
}

}

}
