#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include <tue/config/configuration.h>

#include "tue/control/controller.h"
#include "tue/control/controller_input.h"

namespace tue
{

namespace control
{

// ----------------------------------------------------------------------------------------------------

namespace
{

// Templated helper function for creating controllers of a specific type
template<typename T>
Controller* _createController() { return new T; }

}

// ----------------------------------------------------------------------------------------------------

enum ControllerStatus
{
    ERROR,
    UNINITIALIZED,
    HOMING,
    READY
};

// ----------------------------------------------------------------------------------------------------

struct ControllerData
{
    ControllerData() : status(UNINITIALIZED), measurement_offset(0), zero_measurement_set(false),
    max_error(INVALID_DOUBLE), output_saturation(INVALID_DOUBLE) {}

    // - - - - - - - - - - - - - - - - - - - - - - - -

    std::string name;
    Controller* controller;
    ControllerStatus status;

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Input

    ControllerInput input;
    double corrected_measurement;

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Output

    ControllerOutput output;

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Measurement correction

    double measurement_offset;

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Safety

    double max_error;
    double output_saturation;

    // - - - - - - - - - - - - - - - - - - - - - - - -
    // Homing

    bool homing_pos_initialized;
    double homing_pos;
    double homing_vel;
    double homing_max_vel;
    double homing_max_acc;
    double zero_measurement;
    bool zero_measurement_set;

    // Homing preconditions
    std::vector<std::string> precondition_homed_joints;
    std::vector<unsigned int> precondition_homed_joint_idx;
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

    /// Configure the controller manager
    void configure(tue::Configuration& config);

    /// Calls the update step for each controller this manager manages.
    void update();


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Controller setters

    /// Set the measurement for the controller with the given index
    void setMeasurement(unsigned int idx, double m) { controllers_[idx].input.measurement = m; }

    /// Set the reference for the controller with the given index
    void setReference(unsigned int idx, double pos, double vel = 0, double acc = 0)
    {
        ControllerData& c = controllers_[idx];
        c.input.pos_reference = pos;
        c.input.vel_reference = vel;
        c.input.acc_reference = acc;
    }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Controller getters

    unsigned int getNumControllers() const { return controllers_.size(); }

    /// Get the output of the controller with the given index
    double getOutput(unsigned int idx) const { return controllers_[idx].output.value; }

    /// Get the state of the controller with the given index
    ControllerStatus getStatus(unsigned int idx) const { return controllers_[idx].status; }

    double getMeasurement(unsigned int idx) const { return controllers_[idx].corrected_measurement; }

    double getError(unsigned int idx) const { return controllers_[idx].output.error; }

    const std::string& getName(unsigned int idx) const { return controllers_[idx].name; }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Homing

    void startHoming(unsigned int idx)
    {
        controllers_[idx].status = HOMING;
        controllers_[idx].homing_pos_initialized = false;
        controllers_[idx].zero_measurement_set = false;
    }

    void stopHoming(unsigned int idx)
    {
        controllers_[idx].status = UNINITIALIZED;
    }

    void setZeroMeasurement(unsigned int idx, double v)
    {
        controllers_[idx].zero_measurement = v;
        controllers_[idx].zero_measurement_set = true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    void setError(unsigned int idx) { controllers_[idx].status = ERROR; }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    /// Given the controller name, looks up the index of the controller. Returns -1 if it does not exist
    int getControllerIdx(const std::string& name) const
    {
        std::map<std::string, unsigned int>::const_iterator it = name_to_controller_idx_.find(name);
        if (it == name_to_controller_idx_.end())
            return -1;
        else
            return it->second;
    }

    /// Register a new type of controller. The controller must derive from 'Controller'. Parameter
    /// 'name' determines the name of the controller type.
    template<typename T>
    void registerControllerType(const std::string& name)
    {
        controller_types_[name] = _createController<T>;
    }

    double dt() const { return dt_; }

private:

    double dt_;

    /// List of controllers, inluding some meta-data
    std::vector<ControllerData> controllers_;

    /// Mapping from controller names to their indices in 'controllers_'
    std::map<std::string, unsigned int> name_to_controller_idx_;

    /// Controller creator function pointer type definition
    typedef Controller* (*t_controller_creator)();

    /// Mapping from controller types to function pointers that create a controller of this type
    std::map<std::string, t_controller_creator> controller_types_;

    /// Given a 'type', constructs a controller of this type if it is registered. If not, returns 0
    Controller* createController(const std::string& type)
    {
        std::map<std::string, t_controller_creator>::const_iterator it = controller_types_.find(type);
        if (it == controller_types_.end())
            return NULL;

        t_controller_creator creator = it->second;
        return creator();
    }

    // This object is non-copyable
    ControllerManager(const ControllerManager&) {}
    ControllerManager& operator=(const ControllerManager&);


};

}

}

#endif // CONTROLLER_MANAGER_H
