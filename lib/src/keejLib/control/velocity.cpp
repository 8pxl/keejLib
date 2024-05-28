#pragma once
#include "keejLib/lib.hpp"
#include <numeric>

namespace keejLib
{
    VelocityController::VelocityController(pros::MotorGroup* mtrs, PIDConstants constants, double ka): mtrs(mtrs), pid(PID(constants)), ema(EMA(ka)) {}
    
    void VelocityController::setVelocity(double v) {target = v;}
    
    void VelocityController::applyVoltage() {
        std::vector<double> v = (mtrs -> get_actual_velocities());
        double curr((std::reduce(v.begin(), v.end()) / v.size()));
        double error = target - curr;
        mtrs -> move_voltage(pid.out(error));
    }
} 