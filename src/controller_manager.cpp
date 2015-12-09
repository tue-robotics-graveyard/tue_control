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
    registerControllerType<GenericController>("generic");
    registerControllerType<SetpointController>("setpoint");
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
    dt_;
    if (!config.value("dt", dt_))
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

        Controller* c = createController(type);
        if (!c)
        {
            config.addError("Unknown controller type: '" + type + "'");
            continue;
        }

        c->configure(config, dt_);

        name_to_controller_idx_[name] = controllers_.size();
        controllers_.push_back(ControllerData());
        ControllerData& data = controllers_.back();
        data.controller = c;
        data.name = name;

        if (config.readGroup("homing"))
        {
            data.homed = true;
            data.state = UNINITIALIZED;
            config.value("velocity", data.homing_max_vel);
            config.value("acceleration", data.homing_max_acc);
            data.homing_max_acc = std::abs(data.homing_max_acc);
        }
        else
        {
            data.homed = true;
            data.state = READY;
        }
    }

    config.endArrayItem();
}

// ----------------------------------------------------------------------------------------------------

void ControllerManager::update()
{
    for(std::vector<ControllerData>::iterator it = controllers_.begin(); it != controllers_.end(); ++it)
    {
        ControllerData& c = *it;
        c.output = 0;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        if (!is_set(c.input.measurement))
        {
            std::cout << "Input not set for controller '" << c.name << "'" << std::endl;
            continue;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Handle error state

        if (c.state == ERROR)
        {
            continue;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check if zero measurement is set

        if (c.zero_measurement_set)
        {
            c.measurement_offset = c.zero_measurement - c.input.measurement;
            c.zero_measurement_set = false;
            c.state = READY;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Homing

        if (c.state == HOMING)
        {
            if (!c.homing_pos_initialized)
            {
                // First time we enter the homing loop, so set current position based on measurement
                // and velocity to 0
                c.homing_pos = c.input.measurement;
                c.homing_vel = 0;
                c.homing_pos_initialized = true;
            }
            else
            {
                // Determine homing direction based on max_vel sign
                double dir = c.homing_max_vel < 0 ? -1 : 1;

                // Determine absolute max velocity
                double abs_vel_max = std::abs(c.homing_max_vel);

                // Set homing acceleration based on direction
                c.input.acc_reference = dir * c.homing_max_acc;

                // Increase (or decrease if acc < 0) velocity based on acceleration
                c.homing_vel += dt_ * c.input.acc_reference;

                // Clip velocity between -vel_max and +vel_max
                c.homing_vel = std::min(std::max(c.homing_vel, -abs_vel_max), abs_vel_max);

                // Increase (or decrease if vel < 0) position based on velocity
                c.homing_pos += dt_ * c.homing_vel;
            }

            // Set as set-point for controller
            c.input.pos_reference = c.homing_pos;
            c.input.vel_reference = c.homing_vel;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Apply measurement correction

        c.input.measurement += c.measurement_offset;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Update controller

        c.controller->update(c.input);
        c.output = c.controller->getOutput();

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        // Invalidate measurement such that we can check if the user sets it in the next cycle
        c.input.measurement = INVALID_DOUBLE;
    }
}

}

}
