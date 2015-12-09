#include <tue/control/controller_manager.h>

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

    tue::control::ControllerManager manager;
    manager.configure(config);
    if (config.hasError())
    {
        std::cerr << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    unsigned int idx = manager.getControllerIdx("torso");

    Plant torso;
    torso.setMass(-1);
    torso.setPosition(100);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    int t = 0;

    manager.startHoming(idx);

    while(true)
    {
        if (torso.position() >= 100.2)
        {
            manager.setZeroMeasurement(idx, 0.4);
            break;
        }

        manager.setMeasurement(idx, torso.position());
        manager.update();
        torso.update(manager.getOutput(idx), manager.dt());

        if (t % 100 == 0)
            std::cout << "[" << manager.dt() * t << "] controller output = " << manager.getOutput(idx) << ", measurement = " << manager.getMeasurement(idx) << std::endl;

        ++t;
    }

    manager.stopHoming(idx);

    std::cout << "-------------------------------------------------------------" << std::endl;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    manager.setReference(idx, 0.1);

    while(true)
    {
        manager.setMeasurement(idx, torso.position());
        manager.update();
        torso.update(manager.getOutput(idx), manager.dt());

        if (t % 100 == 0)
            std::cout << "[" << manager.dt() * t << "] controller output = " << manager.getOutput(idx) << ", measurement = " << manager.getMeasurement(idx) << std::endl;

        if (t > 30000)
            break;

        ++t;
    }

    return 0;
}
