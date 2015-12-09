#ifndef GENERICCONTROLLER_H
#define GENERICCONTROLLER_H

#include "controller.h"

#include <scl/filters/DFirstOrderLowpass.hpp>
#include <scl/filters/DLeadLag.hpp>
#include <scl/filters/DPD.hpp>
#include <scl/filters/DPID.hpp>
#include <scl/filters/DSecondOrderLowpass.hpp>
#include <scl/filters/DSkewedNotch.hpp>
#include <scl/filters/DWeakIntegrator.hpp>

namespace tue
{

namespace control
{

/// Filters
struct Filters
{
    Filters() : first_order_low_pass(0), lead_lag(0), PD(0), PID(0), second_order_low_pass(0), skewed_notch(0), weak_integrator(0) {}
    ~Filters() { clear(); }

    void clear()
    {
        delete first_order_low_pass;
        delete lead_lag;
        delete PD;
        delete PID;
        delete second_order_low_pass;
        delete skewed_notch;
        delete weak_integrator;
    }

    DFILTERS::DFirstOrderLowpass* first_order_low_pass;
    DFILTERS::DLeadLag* lead_lag;
    DFILTERS::DPD* PD;
    DFILTERS::DPID* PID;
    DFILTERS::DSecondOrderLowpass* second_order_low_pass;
    DFILTERS::DSkewedNotch* skewed_notch;
    DFILTERS::DWeakIntegrator* weak_integrator;
};

class GenericController : public Controller
{
public:

    /// Default constructor
    /**
    Constructor for the controller
    */
    GenericController();

    /// Destructor
    /**
    Destructor that finalizes, i.e. resets parameters of the controller
    */
    ~GenericController();

    /// Controller configuration
    /**
    Function used to configure the specific controller, it contains the set-up
    of the feedforward and the selected filters
    @param config The configuration of the controller
    @param sample_time The sample time of the controller
    */
    void configure(tue::Configuration &config, double dt);

    /// Controller update
    /**
    Function used for update of the current controller output,
    depending on the given current input
    @param measurement measurement provided by the sensor
    @param reference provided by the user
    */
    void update(const ControllerInput& input, ControllerOutput& output);

protected:

    double gain_;
    Filters filters_;

};

}

}

#endif // GENERICCONTROLLER_H
