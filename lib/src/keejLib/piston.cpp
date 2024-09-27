#include "keejLib/lib.h"

keejLib::Pis::Pis(std::vector<pros::ADIDigitalOut> p, bool s) : pistons(p), state(s)
{
    setState(s);
}

void keejLib::Pis::toggle()  {
    state = !state;

    for(int i = 0; i < pistons.size(); i++) {
        pistons[i].set_value(state);
    }
}

void keejLib::Pis::setState(bool iState) {
    state = iState;

    for(int i = 0; i < pistons.size(); i++)
    {
        pistons[i].set_value(state);
    }
}

bool keejLib::Pis::getState()
{
    return(state);
}