#include "tue/control/controller_factory.h"

#include "tue/control/controller.h"
#include "tue/control/supervised_controller.h"

namespace tue
{
namespace control
{

// ----------------------------------------------------------------------------------------------------

ControllerFactory::ControllerFactory()
{
}

// ----------------------------------------------------------------------------------------------------

ControllerFactory::~ControllerFactory()
{
}

// ----------------------------------------------------------------------------------------------------

std::shared_ptr<SupervisedController> ControllerFactory::createController(tue::Configuration& config, double dt) const
{
    std::shared_ptr<SupervisedController> supervised_controller;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Get controller name and type

    std::string name, type;
    if (!config.value("name", name) | !config.value("type", type))
        return supervised_controller;

    config.setShortErrorContext(name);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Create controller core from given type

    std::map<std::string, t_controller_creator>::const_iterator it = controller_types_.find(type);
    if (it == controller_types_.end())
    {
        config.addError("Unknown controller type: '" + type + "'");
        return supervised_controller;
    }

    std::shared_ptr<Controller> c = it->second();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure controller core

    c->configure(config, dt);
    c->setName(name);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Wrap controller in supervised controller, and configure it

    supervised_controller.reset(new SupervisedController);
    supervised_controller->setController(c);
    supervised_controller->configure(config, dt);

    return supervised_controller;
}

} // end namespace tue

} // end namespace control

