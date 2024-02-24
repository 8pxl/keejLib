#include "main.h"
#include "robot.hpp"
#include "controls.hpp"
#include "autons.hpp"
#include "keejLib/lib.hpp"

// - globals
void (*auton)();
bool color;

void initialize() 
{	glb::imu.reset();
	//example usages of controller.select (use arrow keys to select)
	auton = autons.autonsList[robot::controller.select(autons.names)];	
	color = robot::controller.select({"blue", "red"}); //0 = blue, 1 = red
}

void autonomous() {auton();}

void opcontrol() 
{
	pros::ADIDigitalOut p1('A');
	pros::ADIDigitalOut p2('B');
	lib::pis pistons({p1,p2}, true);

	pistons.toggle();
	if (pistons.getState()) {
		pistons.setState(false);
	}

	while (true) 
	{
		driver();
		pros::delay(20);
	}
}