#include "main.h"
#include "robot.hpp"
// #include "util.hpp"

using namespace lib;
void driver()
{
    std::vector<bool> button = robot::controller.getAll(ALLBUTTONS);
    robot::chassMtrs.spinDiffy(robot::controller.drive(1, lib::controller::arcade));

    if(button[L1]) std::cout << "button L1 was pressed!" << std::endl;
    if(button[NL1]) std::cout << "new button L1 press detected!" << std::endl;
    
    if(button[A]) robot::chassMtrs.spin(127);
    else robot::chassMtrs.stop('c');
}