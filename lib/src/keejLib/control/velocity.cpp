#pragma once
#include "keejLib/lib.hpp"

namespace keejLib
{
    VelocityController::VelocityController(pros::MotorGroup* mtrs, PidConstants constants): mtrs(mtrs), controller(Pid(constants)) {
        controller = Pid(constants);
    }
} 