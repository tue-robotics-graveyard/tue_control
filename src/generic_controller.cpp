#include "tue/control/generic_controller.h"

namespace tue
{

namespace control
{

GenericController::GenericController() : gain_(0), filters_(Filters()),
    ffw_gravity_(0), ffw_static_(0), ffw_dynamic_(0), ffw_acceleration_(0), ffw_direction_(0)
{

}

GenericController::~GenericController()
{
}

void GenericController::configure(tue::Configuration& config, double dt)
{
    //! Clear the filters (required for reconfiguring)
    filters_.clear();

    //! Get the gain
    config.value("gain", gain_);

    //! Get the filters
    if (config.readGroup("filters"))
    {
        if (config.readGroup("weak_integrator"))
        {
            double fz;
            config.value("fz", fz);

            if (fz < 0)
                config.addError("fz < 0");

            if (!config.hasError())
                filters_.weak_integrator = new DFILTERS::DWeakIntegrator(fz, dt);

            config.endGroup();
        }

        if (config.readGroup("lead_lag"))
        {
            double fz, fp;
            config.value("fz", fz);
            config.value("fp", fp);

            if (fz < 0 || fp < 0)
                config.addError("fz < 0 || fp < 0");

            if (!config.hasError())
                filters_.lead_lag = new DFILTERS::DLeadLag(fz, fp, dt);

            config.endGroup();
        }

        if (config.readGroup("skewed_notch"))
        {
            double fz, dz, fp, dp;
            config.value("fz", fz);
            config.value("dz", dz);
            config.value("fp", fp);
            config.value("dp", dp);

            if (fz < 0 || dz < 0 || fp < 0 || dp < 0)
                config.addError("fz < 0 || dz < 0 || fp < 0 || dp < 0");

            if (!config.hasError())
                filters_.skewed_notch = new DFILTERS::DSkewedNotch(fz, dz,fp, dp, dt);

            config.endGroup();
        }

        if (config.readGroup("second_order_low_pass"))
        {
            double fp, dp;
            config.value("fp", fp);
            config.value("dp", dp);

            if (fp < 0 || dp < 0)
                config.addError("fp < 0 || dp < 0");

            if (!config.hasError())
                filters_.second_order_low_pass = new DFILTERS::DSecondOrderLowpass(fp, dp, dt);

            config.endGroup();
        }

        // end filters
        config.endGroup();
    }

    if (config.readGroup("feedforward"))
    {
        config.value("gravity", ffw_gravity_);
        config.value("static", ffw_static_);
        config.value("dynamic", ffw_dynamic_);
        config.value("acceleration", ffw_acceleration_);

        if (!config.value("direction", ffw_direction_, tue::OPTIONAL))
            ffw_direction_ = 1;

        config.endGroup(); // end feedforward
    }
}

void GenericController::update(const ControllerInput& input, ControllerOutput& output)
{
    if (!is_set(input.pos_reference) || !is_set(input.measurement))
        return;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //! 1) Calculate the error

    double error = input.pos_reference - input.measurement;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //! 2) Apply gain

    double out = gain_ * error;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //! 3) Check what filters are configured and apply these

    // Apply weak_integrator
    if (filters_.weak_integrator)
    {
        filters_.weak_integrator->update( out );
        out = filters_.weak_integrator->getOutput();
    }

    // Apply lead_lag
    if (filters_.lead_lag)
    {
        filters_.lead_lag->update( out );
        out = filters_.lead_lag->getOutput();
    }

    // Apply skewed_notch
    if (filters_.skewed_notch)
    {
        filters_.lead_lag->update( out );
        out = filters_.lead_lag->getOutput();
    }

    // Apply second_order_low_pass
    if (filters_.second_order_low_pass)
    {
        filters_.second_order_low_pass->update( out );
        out = filters_.second_order_low_pass->getOutput();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    //! 4) Apply feed forward

    double ff = ffw_gravity_;
    if (is_set(input.vel_reference))
    {
        double vel_sign = input.vel_reference < 0 ? -1 : (input.vel_reference > 0 ? 1 : 0);
        ff += ffw_static_ * vel_sign + ffw_dynamic_ * input.vel_reference;
    }

    if (is_set(input.acc_reference))
        ff += ffw_acceleration_ * input.acc_reference;

    out += ffw_direction_ * ff;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set output

    output.value = out;
    output.error = error;

    return;
}

}

}

