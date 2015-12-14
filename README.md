# Overview

Classes:

    Controller:
        
        Base controller class. Given a measurement and reference calculates output.

    SupervisedController:
        
        Wraps Controller with homing and safety. Contains state machine with following states:

            UNINITIALIZED: start state. 

            HOMING: controller is homing

            ACTIVE: controller is active and will follow given reference

            INACTIVE: controller no longer follows reference (e.g. usefull to switch to if
                      emergence switch is pressed)

            ERROR: controller is in error

    ControllerFactory:

        Generates SupervisedController from a given (tue_config) configuration.

    GenericController:

        Implementation of Controller. Contains multiple configurable filters.

    SetpointController:

        Implementation of Controller. Sets given input directly as output (e.g. usefull for
        dynamixel control)

How to use: see 'test/test_controller.cpp'
