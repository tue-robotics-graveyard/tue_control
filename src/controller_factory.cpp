#include "tue/control/controller_factory.h"

#include "tue/control/controller.h"

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

std::shared_ptr<SupervisedController> ControllerFactory::createController(tue::Configuration& config) const
{
    std::shared_ptr<SupervisedController> controller;

//    std::string name, type;
//    if (!config.value("name", name) | !config.value("type", type))
//        continue;

//    config.setShortErrorContext(name);

//    if (name_to_controller_idx_.find(name) != name_to_controller_idx_.end())
//    {
//        config.addError("Controller with name '" + name + "' already exists.");
//        continue;
//    }

//    Controller* c = createController(type);
//    if (!c)
//    {
//        config.addError("Unknown controller type: '" + type + "'");
//        continue;
//    }

//    c->configure(config, dt_);

//    name_to_controller_idx_[name] = controllers_.size();
//    controllers_.push_back(ControllerData());
//    ControllerData& data = controllers_.back();
//    data.controller = c;
//    data.name = name;

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    // Configure safety

//    if (config.readGroup("safety"))
//    {
//        config.value("output_saturation", data.output_saturation, tue::OPTIONAL);
//        config.value("max_error", data.max_error, tue::OPTIONAL);
//        config.endGroup(); // End safety
//    }

//    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//    // Configure homing

//    if (config.readGroup("homing"))
//    {
//        data.status = UNINITIALIZED;
//        config.value("velocity", data.homing_max_vel);
//        config.value("acceleration", data.homing_max_acc);
//        data.homing_max_acc = std::abs(data.homing_max_acc);

//        // Read preconditions config
//        if (config.readArray("preconditions"))
//        {
//            while(config.nextArrayItem())
//            {
//                std::string homed_joint;
//                if (config.value("homed", homed_joint, tue::OPTIONAL))
//                {
//                    data.precondition_homed_joints.push_back(homed_joint);
//                }
//                else
//                {
//                    std::stringstream s;
//                    s << config;
//                    config.addError("Unknown precondition: " + s.str());
//                }

//            }

//            config.endArray();
//        }

//        config.endGroup();
//    }
//    else
//    {
//        data.status = READY;
//    }

    return controller;
}

} // end namespace tue

} // end namespace control

