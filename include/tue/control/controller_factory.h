#ifndef TUE_CONTROL_CONTROLLER_FACTORY_H_
#define TUE_CONTROL_CONTROLLER_FACTORY_H_

#include <memory>

#include <tue/config/configuration.h>

namespace tue
{

namespace control
{

class Controller;
class SupervisedController;

// ----------------------------------------------------------------------------------------------------

namespace
{

// Templated helper function for creating controllers of a specific type
template<typename T>
Controller* _createController() { return new T; }

}

// ----------------------------------------------------------------------------------------------------

class ControllerFactory
{

public:

    ControllerFactory();

    ~ControllerFactory();

    std::shared_ptr<SupervisedController> createController(tue::Configuration& config) const;

    /// Register a new type of controller. The controller must derive from 'Controller'. Parameter
    /// 'name' determines the name of the controller type.
    template<typename T>
    void registerControllerType(const std::string& name)
    {
        controller_types_[name] = _createController<T>;
    }

private:

    /// Controller creator function pointer type definition
    typedef Controller* (*t_controller_creator)();

    /// Mapping from controller types to function pointers that create a controller of this type
    std::map<std::string, t_controller_creator> controller_types_;

};

} // end namespace tue

} // end namespace control

#endif
