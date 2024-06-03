#include "keejLib/lib.h"

using namespace keejLib;

std::pair<double, double> Chassis::pidMTPVel(pt target, double rotationBias, PID *lCont, PID *rCont) {
    double linearError = pose.pos.dist(target);
    double currHeading = imu -> get_heading();
    double targetHeading = absoluteAngleToPoint(pos, target);
    double rotationError = lib::minError(targetHeading,currHeading);
    double cre = abs(rotationError) > 90 ? 0.1 : cos(lib::dtr(rotationError));
    double angularVel = rCont -> out(rotationError);
    double linearVel = cre * lCont -> out(linearError);
    double rVel = (linearVel - (fabs(angularVel) * rotationBias)) + angularVel;
    double lVel = (linearVel - (fabs(angularVel) * rotationBias)) - angularVel;
    return({lVel, rVel});
}

void Chassis::driveAngle(double dist, double angle, MotionParams params) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { driveAngle(dist, angle, params);});
        pros::delay(10);
        return;
    }
    Exit timeout = exit::Timeout(params.timeout);
    PID linCont = PID(this -> linConsts);
    PID angCont = PID(this -> angConsts);
    double linError;
    this -> dt -> tare_position();
    while (params.exit.exited({.error = linError}) || timeout.exited()) {
        linError = dist - (this -> dt -> getAvgPosition());
        double angularError = angError(angle, imu -> get_heading());
    
        if (std::abs(angularError) < this -> angConsts.tolerance) {
            angularError = 0;
        }
        double va = angCont.out(angularError);
        double vl = linCont.out(linError);
        
        if (std::abs(vl) + std::abs(va) > 127) {
          vl = (127 - std::abs(va)) * sign(vl);
        }
        
        this -> dt -> spinVolts(vl - va, vl + va);
    }
}

void Chassis::mtp(pt target, double theta, double dLead, MotionParams params) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { mtp(target, theta, dLead, params);});
        pros::delay(10);
        return;
    }
    
    Exit timeout = exit::Timeout(params.timeout);
    PID linCont = PID(this -> linConsts);
    PID angCont = PID(this -> angConsts);
    
    while (timeout.exited() || params.exit.exited({}))
}