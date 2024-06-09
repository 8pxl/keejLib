#pragma once
#include "keejLib/lib.h"
#include <numeric>

using namespace keejLib;
    VelocityController::VelocityController(PIDConstants constants, double ka): pid(PID(constants)), ema(EMA(ka)) {}
    
    void VelocityController::setVelocity(double v) {target = v;}
    
    double VelocityController::getVoltage(double curr) {
        double error = target - curr;
        return(pid.out(error));
    }