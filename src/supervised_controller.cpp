#include "tue/control/supervised_controller.h"

#include <tue/control/controller.h>

namespace tue
{
namespace control
{

// ----------------------------------------------------------------------------------------------------

SupervisedController::SupervisedController() : event_(NONE), measurement_offset(0),
    error_(INVALID_DOUBLE), output_(INVALID_DOUBLE), output_saturation_(INVALID_DOUBLE), max_error_(INVALID_DOUBLE)
{
    input_.measurement = INVALID_DOUBLE;
}

// ----------------------------------------------------------------------------------------------------

SupervisedController::~SupervisedController()
{
}

// ----------------------------------------------------------------------------------------------------

void SupervisedController::configure(tue::Configuration& config, double dt)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure safety

    if (config.readGroup("safety"))
    {
        config.value("output_saturation", output_saturation_, tue::OPTIONAL);
        config.value("max_error", max_error_, tue::OPTIONAL);
        config.endGroup(); // End safety
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configure homing

    if (config.readGroup("homing"))
    {
        config.value("velocity", homing_max_vel_);
        config.value("acceleration", homing_max_acc_);
        homing_max_acc_ = std::abs(homing_max_acc_);
        config.endGroup();

        homed_ = false;
        homable_ = true;
    }
    else
    {
        homed_ = true;
        homable_ = false;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    status_ = UNINITIALIZED;

    dt_ = dt;
}

// ----------------------------------------------------------------------------------------------------

void SupervisedController::checkTransitions(double raw_measurement)
{
    ControllerStatus old_status = status_;

    if (event_ == STOP_HOMING && status_ == HOMING)
    {
        measurement_offset = homed_measurement_ - raw_measurement;
        input_.measurement = homed_measurement_;

        homed_ = true;

        // automatically switch to active after homing
        status_ = IDLE;
        event_ = ENABLE;
    }

    if (event_ == START_HOMING && status_ != HOMING && homable_)
    {
        status_ = HOMING;
        homing_pos = raw_measurement;
        homing_vel = 0;
    }
    else if (event_ == ENABLE && status_ != ACTIVE && homed_ == true)
    {
        status_ = ACTIVE;

        input_.pos_reference = input_.measurement;
        input_.vel_reference = 0;
        input_.acc_reference = 0;

        std::cout << "Controller activated: reset ref to " << input_.pos_reference << std::endl;

    }
    else if (event_ == SET_ERROR)
    {
        status_ = ERROR;
    }
    else if (event_ == DISABLE)
    {
        status_ = IDLE;
    }

    if (old_status == ACTIVE && status_ != ACTIVE)
    {
        // Just switched to not being active.
        // TODO: reset controllers
    }

    event_ = NONE;
}

// ----------------------------------------------------------------------------------------------------

void SupervisedController::update(double raw_measurement)
{
    output_ = 0;

    if (!is_set(raw_measurement)) // TODO
        return;

    if (status_ == UNINITIALIZED)
        status_ = IDLE;

    input_.measurement = raw_measurement + measurement_offset;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    checkTransitions(raw_measurement);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Update controller

    ControllerOutput output;
    output.value = 0;
    output.error = INVALID_DOUBLE;

    switch (status())
    {

    case HOMING:
    {
        if (!is_set(raw_measurement))
            setError("While homing: no or bad measurement received");
        else
            updateHoming(raw_measurement, output);
        break;
    }

    case ACTIVE:
    {
        if (!is_set(raw_measurement))
            setError("While active: no or bad measurement received");
        else
            controller_->update(input_, output);
        break;
    }

    default:
        break;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    error_ = output.error;
    output_ = output.value;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check safety

    if (!is_set(output_))
    {
        setError("Invalid output");
    }
    else if (is_set(error_) && std::abs(error_) > max_error_)
    {
        setError("Max error reached");
    }
    else if (is_set(output_saturation_))   // Output saturation
    {
        if (output_ < -output_saturation_)
            output_ = -output_saturation_;
        else if (output_ > output_saturation_)
            output_ = output_saturation_;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // We may have an SET_ERROR event, so re-check transitions

    checkTransitions(raw_measurement);
}

// ----------------------------------------------------------------------------------------------------

void SupervisedController::updateHoming(double measurement, ControllerOutput& output)
{
    ControllerInput homing_input;
    homing_input.measurement = measurement;

    // Determine homing direction based on max_vel sign
    double dir = homing_max_vel_ < 0 ? -1 : 1;

    // Determine absolute max velocity
    double abs_vel_max = std::abs(homing_max_vel_);

    // Set homing acceleration based on direction
    homing_input.acc_reference = dir * homing_max_acc_;

    // Increase (or decrease if acc < 0) velocity based on acceleration
    homing_vel += dt_ * homing_input.acc_reference;

    // Clip velocity between -vel_max and +vel_max
    homing_vel = std::min(std::max(homing_vel, -abs_vel_max), abs_vel_max);

    // Increase (or decrease if vel < 0) position based on velocity
    homing_pos += dt_ * homing_vel;

    // Set as set-point for controller
    homing_input.pos_reference = homing_pos;
    homing_input.vel_reference = homing_vel;

    // Update controller
    controller_->update(homing_input, output);
}

// ----------------------------------------------------------------------------------------------------

const std::string& SupervisedController::name() const
{
    return controller_->name();
}

// ----------------------------------------------------------------------------------------------------

} // end namespace tue

} // end namespace control

