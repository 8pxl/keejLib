#pragma once
#include "keejLib/lib.hpp"
#include "keejLib/units.h"
#include <numeric>

using namespace units::angular_velocity;

namespace keejLib
{
    VelocityController::VelocityController(pros::MotorGroup* mtrs, PIDConstants constants, double ka): mtrs(mtrs), pid(PID(constants)), ema(EMA(ka)) {}
    
    void VelocityController::setVelocity(revolutions_per_minute_t v) {target = v;}
    
    void VelocityController::applyVoltage() {
        std::vector<double> v = (mtrs -> get_actual_velocities());
        revolutions_per_minute_t curr((std::reduce(v.begin(), v.end()) / v.size()));
        revolutions_per_minute_t error = target - curr;
        mtrs -> move_voltage(pid.out(unit_cast<double>(error)));
    }
} 