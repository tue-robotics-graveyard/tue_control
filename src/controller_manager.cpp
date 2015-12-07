#include "tue/control/controller_manager.h"

#include "tue/control/generic_controller.h"
#include "tue/control/setpoint_controller.h"

namespace tue
{

namespace control
{

// ----------------------------------------------------------------------------------------------------

ControllerManager::ControllerManager()
{

}

// ----------------------------------------------------------------------------------------------------

ControllerManager::~ControllerManager()
{
    for(std::vector<ControllerData>::iterator it = controllers_.begin(); it != controllers_.end(); ++it)
        delete it->controller;
}

// ----------------------------------------------------------------------------------------------------

void ControllerManager::configure(tue::Configuration& config)
{
    double sample_time;
    if (!config.value("sample_time", sample_time))
        return;

    if (!config.readArray("controllers", tue::REQUIRED))
        return;

    while(config.nextArrayItem())
    {
        std::string name, type;
        if (!config.value("name", name) | !config.value("type", type))
            continue;

        config.setShortErrorContext(name);

        if (name_to_controller_idx_.find(name) != name_to_controller_idx_.end())
        {
            config.addError("Controller with name '" + name + "' already exists.");
            continue;
        }

        Controller* c = NULL;
        if (type == "generic")
            c = new GenericController;
        else if (type == "setpoint")
            c = new SetpointController;
        else
        {
            config.addError("Unknown controller type: '" + type + "'");
            continue;
        }

        c->configure(config, sample_time);

        name_to_controller_idx_[name] = controllers_.size();
        controllers_.push_back(ControllerData());
        ControllerData& data = controllers_.back();
        data.controller = c;
        data.name = name;
    }

    config.endArrayItem();
}

// ----------------------------------------------------------------------------------------------------

void ControllerManager::update()
{
    for(std::vector<ControllerData>::iterator it = controllers_.begin(); it != controllers_.end(); ++it)
    {
        ControllerData& c = *it;
        c.controller->update(c.input);
    }
}

// ----------------------------------------------------------------------------------------------------

double ControllerManager::getOutput(unsigned int idx) const
{
    return controllers_[idx].controller->getOutput();
}

}

}
