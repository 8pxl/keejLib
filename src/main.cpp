#include "main.h"
#include "lib.h"

// - globals
void (*auton)();

void initialize() 
{
}

// void autonomous() {auton();}

void opcontrol() 
{
	while (true) 
	{
		pros::delay(20);
	}
}