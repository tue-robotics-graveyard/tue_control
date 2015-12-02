#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include <tue/config/configuration.h>

namespace tue
{

namespace control
{

class ControllerManager
{
public:

    /// Default constructor
    /**
    Constructor for the controller manager
    */
    ControllerManager();

    /// Destructor
    /**
    Destructor that finalizes, i.e. resets parameters of the controller manager and its controllers
    */
    ~ControllerManager();

protected:


};

}

}

#endif // CONTROLLER_MANAGER_H
