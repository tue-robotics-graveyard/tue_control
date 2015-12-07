#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include <tue/config/configuration.h>

#include "tue/control/controller_input.h"

namespace tue
{

namespace control
{

class Controller;

struct ControllerData
{
    std::string name;
    Controller* controller;
    ControllerInput input;
    double output;
};

// ----------------------------------------------------------------------------------------------------

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

    void configure(tue::Configuration& config);

    void update();

    void setInput(unsigned int idx, const ControllerInput& input) { controllers_[idx].input = input; }

    double getOutput(unsigned int idx) const;

    int getControllerIdx(const std::string& name) const
    {
        std::map<std::string, unsigned int>::const_iterator it = name_to_controller_idx_.find(name);
        if (it == name_to_controller_idx_.end())
            return -1;
        else
            return it->second;
    }

protected:

    std::vector<ControllerData> controllers_;

    std::map<std::string, unsigned int> name_to_controller_idx_;

private:

    // This object is non-copyable
    ControllerManager(const ControllerManager&) {}
    ControllerManager& operator=(const ControllerManager&);


};

}

}

#endif // CONTROLLER_MANAGER_H
