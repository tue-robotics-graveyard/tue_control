#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <tue/config/configuration.h>

#include "tue/control/controller_input.h"
#include "tue/control/controller_output.h"

namespace tue
{

namespace control
{

class Controller
{
public:

    /// Default constructor
    /**
    Constructor for the controller
    */
    Controller();

    /// Destructor
    /**
    Destructor that finalizes, i.e. resets parameters of the controller
    */
    ~Controller();

    /// Controller configuration
    /**
    Function used to configure the specific controller
    @param config The configuration of the controller
    */
    virtual void configure(tue::Configuration& config, double dt) = 0;

    /// Controller update
    /**
    Function used for update of the current controller output,
    depending on the given current input
    @param measurement measurement provided by the sensor
    @param reference provided by the user
    @param feed_forward (optional) feed_forward parameter that is added to the output
    */
    virtual void update(const ControllerInput& input, ControllerOutput& output) = 0;

};

}

}

#endif // CONTROLLER_H
