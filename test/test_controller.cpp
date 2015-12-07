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
    tue::control::ControllerInput input;
    input.pos_reference = 0.41;
    input.pos_measurement = 0.1;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(unsigned int i = 0; i < 100; ++i)
    {
        manager.setInput(idx, input);
        manager.update();



        std::cout << manager.getOutput(idx) << std::endl;
    }

    return 0;
}
