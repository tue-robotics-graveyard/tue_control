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

using namespace DFILTERS;

/// Filters
struct Filters {
    Filters() : first_order_low_pass(0), lead_lag(0), PD(0), PID(0), second_order_low_pass(0), skewed_notch(0), weak_integrator(0) {}
    ~Filters() { clear(); }

    void clear() {
        if (first_order_low_pass) delete first_order_low_pass;
        if (lead_lag) delete lead_lag;
        if (PD) delete PD;
        if (PID) delete PID;
        if (second_order_low_pass) delete second_order_low_pass;
        if (skewed_notch) delete skewed_notch;
        if (weak_integrator) delete weak_integrator;
    }

    DFirstOrderLowpass* first_order_low_pass;
    DLeadLag* lead_lag;
    DPD* PD;
    DPID* PID;
    DSecondOrderLowpass* second_order_low_pass;
    DSkewedNotch* skewed_notch;
    DWeakIntegrator* weak_integrator;
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
    void configure(tue::Configuration &config, double sample_time);

    /// Controller update
    /**
    Function used for update of the current controller output,
    depending on the given current input
    @param measurement measurement provided by the sensor
    @param reference provided by the user
    */
    void update(const ControllerInput& input);

protected:

    double gain_;
    Filters filters_;

};

}

}

#endif // GENERICCONTROLLER_H
