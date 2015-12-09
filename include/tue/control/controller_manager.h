#ifndef CONTROLLER_MANAGER_H
#define CONTROLLER_MANAGER_H

#include <tue/config/configuration.h>

#include "tue/control/controller_input.h"

namespace tue
{

namespace control
{

// ----------------------------------------------------------------------------------------------------

class Controller;

namespace
{

// Templated helper function for creating controllers of a specific type
template<typename T>
Controller* _createController() { return new T; }

}

// ----------------------------------------------------------------------------------------------------

enum ControllerState
{
    ERROR,
    UNINITIALIZED,
    HOMING,
    READY
};

// ----------------------------------------------------------------------------------------------------

struct ControllerData
{
    ControllerData() : state(UNINITIALIZED), measurement_offset(0), zero_measurement_set(false) {}

    std::string name;
    Controller* controller;
    ControllerState state;
    ControllerInput input;
    double output;
    double corrected_measurement;

    double measurement_offset;

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

    /// Set the input for the controller with the given index
    void setInput(unsigned int idx, const ControllerInput& input) { controllers_[idx].input = input; }

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

    /// Get the output of the controller with the given index
    double getOutput(unsigned int idx) const { return controllers_[idx].output; }

    /// Get the state of the controller with the given index
    ControllerState getState(unsigned int idx) const { return controllers_[idx].state; }

    double getMeasurement(unsigned int idx) { return controllers_[idx].corrected_measurement; }


    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Homing

    void startHoming(unsigned int idx)
    {
        controllers_[idx].state = HOMING;
        controllers_[idx].homing_pos_initialized = false;
        controllers_[idx].zero_measurement_set = false;
    }

    void stopHoming(unsigned int idx)
    {
        controllers_[idx].state = UNINITIALIZED;
    }

    void setZeroMeasurement(unsigned int idx, double v)
    {
        controllers_[idx].zero_measurement = v;
        controllers_[idx].zero_measurement_set = true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    void setError(unsigned int idx) { controllers_[idx].state = ERROR; }


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
