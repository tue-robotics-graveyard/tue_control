#include "tue/control/controller_manager.h"

#include "tue/control/generic_controller.h"
#include "tue/control/setpoint_controller.h"

#include <sstream>

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

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Configure safety

        if (config.readGroup("safety"))
        {
            config.value("output_saturation", data.output_saturation, tue::OPTIONAL);
            config.value("max_error", data.max_error, tue::OPTIONAL);
            config.endGroup(); // End safety
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Configure homing

        if (config.readGroup("homing"))
        {
            data.status = UNINITIALIZED;
            config.value("velocity", data.homing_max_vel);
            config.value("acceleration", data.homing_max_acc);
            data.homing_max_acc = std::abs(data.homing_max_acc);

            // Read preconditions config
            if (config.readArray("preconditions"))
            {
                while(config.nextArrayItem())
                {
                    std::string homed_joint;
                    if (config.value("homed", homed_joint, tue::OPTIONAL))
                    {
                        data.precondition_homed_joints.push_back(homed_joint);
                    }
                    else
                    {
                        std::stringstream s;
                        s << config;
                        config.addError("Unknown precondition: " + s.str());
                    }

                }

                config.endArray();
            }

            config.endGroup();
        }
        else
        {
            data.status = READY;
        }
    }

    config.endArrayItem();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(std::vector<ControllerData>::iterator it = controllers_.begin(); it != controllers_.end(); ++it)
    {
        ControllerData& c = *it;

        c.precondition_homed_joint_idx.resize(c.precondition_homed_joints.size());
        for(unsigned int i = 0; i < c.precondition_homed_joints.size(); ++i)
        {
            int idx = getControllerIdx(c.precondition_homed_joints[i]);

            if (idx < 0)
            {
                config.addError("Homing procedure of controller '" + c.name+ "' depends on non-existing controller '"
                                + c.precondition_homed_joints[i] + "'");
            }
            else
            {
                c.precondition_homed_joint_idx[i] = idx;
            }
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void ControllerManager::update()
{
    for(std::vector<ControllerData>::iterator it = controllers_.begin(); it != controllers_.end(); ++it)
    {
        ControllerData& c = *it;
        c.output.value = 0;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check if measurement is set

        if (!is_set(c.input.measurement))
        {
            std::cout << "Measurement not set for controller '" << c.name << "'" << std::endl;
            continue;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check if zero measurement is set

        if (c.zero_measurement_set)
        {
            c.measurement_offset = c.zero_measurement - c.input.measurement;
            c.zero_measurement_set = false;
            c.status = READY;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Apply measurement correction

        c.corrected_measurement = c.input.measurement + c.measurement_offset;

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Do not continue if in error or uninitialized (then controller outputs 0)

        if (c.status == ERROR || c.status == UNINITIALIZED)
        {
            // Invalidate measurement such that we can check if the user sets it in the next cycle
            c.input.measurement = INVALID_DOUBLE;
            continue;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Homing

        if (c.status == HOMING)
        {
            // - - - - - - - - - - - - - - - - - - - - - - - - -
            // Check if all preconditions are met

            bool conditions_met = true;
            for(std::vector<unsigned int>::const_iterator it = c.precondition_homed_joint_idx.begin();
                it != c.precondition_homed_joint_idx.end(); ++it)
            {
                ControllerData& other = controllers_[*it];
                if (other.status != READY)
                {
                    conditions_met = false;
                    break;
                }
            }

            if (!conditions_met)
                continue;

            // - - - - - - - - - - - - - - - - - - - - - - - - -

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
        // Update controller

        c.input.measurement = c.corrected_measurement;

        // Reset output
        c.output = ControllerOutput();

        c.controller->update(c.input, c.output);

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check output validity

        if (!is_set(c.output.value))
        {
            std::cout << "Controller '" + c.name << "' did not properly provide output" << std::endl;
            c.output.value = 0;
        }

        if (!is_set(c.output.error))
        {
            std::cout << "Controller '" + c.name << "' did not properly provide error" << std::endl;
            c.output.value = 0;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        // Check safety

        if (is_set(c.output.error) && c.output.error > c.max_error)
        {

            std::cout << "MAX ERROR" << std::endl;
            c.output.value = 0;
            c.status = ERROR;
        }

        // Output saturation
        if (is_set(c.output_saturation))
        {
            if (c.output.value < -c.output_saturation)
                c.output.value = -c.output_saturation;
            else if (c.output.value > c.output_saturation)
                c.output.value = c.output_saturation;
        }

        // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        // Invalidate measurement such that we can check if the user sets it in the next cycle
        c.input.measurement = INVALID_DOUBLE;
    }
}

}

}
