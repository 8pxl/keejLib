#include "main.h"
#include "keejLib/lib.h"

// - globals
void (*auton)();
bool color;

void initialize() {
    auto dt = keejLib::DriveTrain({1,2,3}, {4,5,6});
}

void autonomous() {auton();}

void opcontrol() {
}