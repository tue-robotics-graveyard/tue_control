#ifndef TUE_CONTROL_SUPERVISED_CONTROLLER_H_
#define TUE_CONTROL_SUPERVISED_CONTROLLER_H_

#include <memory>

#include <tue/config/configuration.h>

namespace tue
{
namespace control
{

class Controller;
struct ControllerInput;
struct ControllerOutput;

// ----------------------------------------------------------------------------------------------------

enum ControllerStatus
{
    ERROR,
    UNINITIALIZED,
    HOMING,
    INACTIVE,
    ACTIVE
};

// ----------------------------------------------------------------------------------------------------

enum ControllerEvent
{
    NONE,
    START_HOMING,
    STOP_HOMING,
    SET_ERROR,
    SET_ACTIVE,
    SET_INACTIVE
};

// ----------------------------------------------------------------------------------------------------

// Wraps Controller with safety and homing functionality

class SupervisedController
{

public:

    SupervisedController();

    ~SupervisedController();

    SupervisedController(const SupervisedController&) = delete;

    SupervisedController& operator=(const SupervisedController&) = delete;


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Configuration

    void setController(const std::shared_ptr<Controller>& controller) { controller_ = controller; }

    void configure(tue::Configuration& config, double dt);


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Update

    void update(const ControllerInput& input, ControllerOutput& output);

    void updateHoming(ControllerOutput& output);


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Change state / status

    void startHoming() { event_ = START_HOMING; }

    void stopHoming(double current_pos) { event_ = STOP_HOMING; zero_measurement = current_pos; }

    void setError(const std::string& error_msg) { event_ = SET_ERROR; error_msg_ = error_msg; }

    void setInactive() { event_ = SET_INACTIVE; }

    void setActive() { event_ = SET_ACTIVE; }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Getters

    ControllerStatus status() const { return status_; }

    double error() const { return error_; }

    double measurement() const { return measurement_; }

    double output() const { return output_; }

    bool accepts_references() const { return status_ == ACTIVE; }


private:

    double dt_;

    ControllerStatus status_;

    ControllerEvent event_;

    std::shared_ptr<Controller> controller_;

    std::string error_msg_;

    double error_;

    double measurement_;

    double output_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Safety

    double output_saturation_;
    double max_error_;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Homing

    double measurement_offset;

    double homing_max_vel_;
    double homing_max_acc_;

    double homing_pos;
    double homing_vel;
    double homing_max_vel;
    double homing_max_acc;
    double zero_measurement;

};

} // end namespace tue

} // end namespace control

#endif
