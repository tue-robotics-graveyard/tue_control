#include "tue/control/controller.h"

namespace tue
{

namespace control
{

Controller::Controller() : output_(0)
{

}

Controller::~Controller()
{

}

double Controller::getOutput()
{
    return output_;
}

}

}
