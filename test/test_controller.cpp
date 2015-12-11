#include <tue/control/controller_factory.h>
#include <tue/control/supervised_controller.h>

#include <tue/control/controller_input.h>

#include <tue/control/generic_controller.h>
#include <tue/control/setpoint_controller.h>

// ----------------------------------------------------------------------------------------------------

class Plant
{

public:

    void setMass(double mass) { mass_ = mass; }

    void setPosition(double pos) { pos_ = pos; vel_ = 0; }

    void update(double f, double dt)
    {
        double a = f / mass_;
        vel_ += dt * a;
        pos_ += dt * vel_;
    }

    double position() const { return pos_; }

private:

    double mass_;
    double pos_;
    double vel_;

};

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Please provide config file" << std::endl;
        return 1;
    }

    tue::Configuration config;
    config.loadFromYAMLFile(argv[1]);
    if (config.hasError())
    {
        std::cerr << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    tue::control::ControllerFactory factory;
    factory.registerControllerType<tue::control::GenericController>("generic");
    factory.registerControllerType<tue::control::SetpointController>("setpoint");

    typedef std::shared_ptr<tue::control::SupervisedController> SupvControllerPtr;

    double dt;
    config.value("dt", dt);

    SupvControllerPtr c = factory.createController(config, dt);

    if (config.hasError())
    {
        std::cerr << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Plant torso;
    torso.setMass(-1);
    torso.setPosition(100);

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "                          HOMING                             " << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    int t = 0;

    c->startHoming();

    while(true)
    {
        if (torso.position() >= 100.2)
        {
            c->stopHoming(0.4);
            break;
        }

        c->update(torso.position());
        torso.update(c->output(), dt);

        if (t % 100 == 0)
            std::cout << "[" << dt * t << "] controller output = " << c->output() << ", measurement = " << c->measurement() << std::endl;

        ++t;
    }

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "                        SET REFERENCE                        " << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    c->setReference(0.1);

    while(true)
    {
        c->update(torso.position());
        torso.update(c->output(), dt);


        if (t % 100 == 0)
            std::cout << "[" << dt * t << "] controller output = " << c->output() << ", measurement = " << c->measurement() << std::endl;

        if (t > 30000)
            break;

        ++t;
    }

    std::cout << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << "                            ERROR                            " << std::endl;
    std::cout << "-------------------------------------------------------------" << std::endl;
    std::cout << std::endl;

    c->setError("Just a test");

    c->update(torso.position());

    std::cout << "[" << dt * t << "] controller output = " << c->output() << ", measurement = " << c->measurement() << std::endl;

    return 0;
}
