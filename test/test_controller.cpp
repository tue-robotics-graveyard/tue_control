#include <tue/control/controller_manager.h>

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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    manager.startHoming(idx);

    for(unsigned int i = 0; i < 100; ++i)
    {
        manager.setMeasurement(idx, 0.1);
        manager.update();
    }

    manager.setZeroMeasurement(idx, 0.1);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    manager.setReference(idx, 0.41, 0, 0);

    for(unsigned int i = 0; i < 100; ++i)
    {
        manager.setMeasurement(idx, 0.1);
        manager.update();

        std::cout << manager.getOutput(idx) << std::endl;
    }

    return 0;
}
