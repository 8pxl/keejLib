#include "main.h"
#include "keejLib/lib.hpp"

// - globals
using namespace lib;

bool puncherToggled = false;

//pros
pros::Controller pros_controller(pros::E_CONTROLLER_MASTER);
pros::ADIDigitalOut anglerPis('A');
pros::ADIDigitalOut intakePis('B');

namespace robot
{
    lib::diffy chassMtrs({9,8,3,-1,-2,-20});
    lib::mtrs intake({-11});
    lib::mtrs puncher({-18});
    lib::controller controller(pros_controller);
    lib::pis tsukasa({intakePis}, false);
    lib::pis angler({anglerPis}, false);
}

void opcontrol() 
{
	while (true) 
	{
		std::vector<bool> state = robot::controller.getAll(ALLBUTTONS);
		robot::chassMtrs.spinDiffy(robot::controller.drive(-1, lib::controller::arcade));
		
		if(state[NR1]) robot::tsukasa.setState(false);
		if(state[R1]) robot::intake.spin(127);
		else if(state[R2]) robot::intake.spin(-127);
		else robot::intake.stop('c');

		if(state[L1]) robot::puncher.spin(127);
		else if(!puncherToggled) robot::puncher.stop('c');
	
		if(state[NL2]) robot::angler.toggle();
		if(state[NUP]) robot::tsukasa.toggle();

		if(puncherToggled) robot::puncher.spin(127);
		if(state[NA]) puncherToggled = !puncherToggled;
		pros::delay(20);
	}
}