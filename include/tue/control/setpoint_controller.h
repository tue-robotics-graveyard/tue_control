#ifndef SETPOINTCONTROLLER_H
#define SETPOINTCONTROLLER_H

#include "controller.h"

namespace tue
{

namespace control
{

class SetpointController : public Controller
{
public:

    /// Default constructor
    /**
    Constructor for the controller
    */
    SetpointController();

    /// Destructor
    /**
    Destructor that finalizes, i.e. resets parameters of the controller
    */
    ~SetpointController();

    /// Controller configuration
    /**
    Function used to configure the specific controller, it contains the set-up
    of the feedforward and the selected filters
    @param config The configuration of the controller
    @param sample_time The sample time of the controller
    */
    void configure(tue::Configuration &config, double dt);

    /// Controller update
    /**
    Function used for update of the current controller output,
    depending on the given current input
    @param measurement measurement provided by the sensor
    @param reference provided by the user
    */
    void update(const ControllerInput& input, ControllerOutput& output);

};

}

}

#endif // SETPOINTCONTROLLER_H
