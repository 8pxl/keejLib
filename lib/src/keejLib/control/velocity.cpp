#pragma once
#include "keejLib/control.h"
#include "keejLib/lib.h"
#include <numeric>

namespace keejLib {
    
VelocityController::VelocityController(PIDConstants constants, double ka): pid(PID(constants)), ema(EMA(ka)) {}

void VelocityController::setVelocity(double v) {target = v;}

double VelocityController::getVoltage(double curr) {
    double error = target - curr;
    return(pid.out(error));
}

ChassVelocities VelocityManager::update(std::pair<double, double> vals) {
    if (slew.has_value()) vl = sign(vals.first) * std::min(fabs(vl) + slew.value(), fabs(vals.first));
    // std::cout << linMin << " " << linMax << " " << angMin << " " << angMax << std::endl;
    vl = sign(vals.first) * std::clamp(fabs(vals.first), linMin, linMax);
    va = sign(vals.second) * std::clamp(fabs(vals.second), angMin, angMax);
    std::cout << "vl: " << vl << " va: " << va << std::endl;
    //desaturate;
    // vl = (127 * (1+ratio)) / (2*ratio);
    // va = 127 - vl;
    if (std::abs(vl) + std::abs(va) > 127) {
      vl = (127 - std::abs(va)) * sign(vl);
    }
    return {vl + va, vl - va};
}
void VelocityManager::setLinMin(double newLinMin) {
    if (newLinMin < linMax) linMin = newLinMin;
}

void VelocityManager::setLinMax(double newLinMax) {
    if (newLinMax > linMin) linMax = newLinMax;
}

void VelocityManager::setAngMin(double newAngMin) {
    if (newAngMin < angMax) angMin = newAngMin;
}

void VelocityManager::setAngMax(double newAngMax) {
    if (newAngMax > angMin) angMax = newAngMax;
}

void VelocityManager::setSlew(double newSlew) {
    slew = newSlew;
}
}