#include "keejLib/lib.h"
#include <cmath>

using namespace keejLib;

std::pair<double, double> Chassis::pidMTPVel(Pt target, MotionParams params, PID *lCont, PID *rCont) {
    double linearError = pose.pos.dist(target);
    Angle currHeading = Angle(imu -> get_heading(), HEADING);
    Angle targetHeading = absoluteAngleToPoint(pose.pos, target);
    double rotationError = targetHeading.error(currHeading);
    double cre = fabs(rotationError) > 90 ? 0.1 : cos(toRad(rotationError));
    double angularVel = rCont -> out(rotationError);
    double linearVel = cre * lCont -> out(linearError);
    
    if (linearVel < params.vMin) {
        linearVel = params.vMin;
    }
    if (std::abs(linearVel) + std::abs(angularVel) > 127) {
        linearVel = (127 - std::abs(angularVel)) * sign(linearVel);
    }
    double rVel = (linearVel - (fabs(angularVel) * params.mtpRotBias)) + angularVel;
    double lVel = (linearVel - (fabs(angularVel) * params.mtpRotBias)) - angularVel;
    return(std::make_pair(lVel, rVel));
}

void Chassis::driveAngle(double dist, double angle, MotionParams params) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { driveAngle(dist, angle, params);});
        pros::delay(10);
        return;
    }
    Angle targ = Angle(angle, HEADING);
    
    Exit timeout = exit::Timeout(params.timeout);
    PID linCont = PID(this -> linConsts);
    PID angCont = PID(this -> angConsts);
    double linError;
    this -> dt -> tare_position();
    while (params.exit.exited({.error = linError}) || timeout.exited()) {
        linError = dist - (this -> dt -> getAvgPosition());
        double angularError = targ.error(Angle(imu -> get_rotation(), HEADING));
    
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

void Chassis::mtp(Pose target, double theta, double dLead, MotionParams params) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { mtp(target, theta, dLead, params);});
        pros::delay(10);
        return;
    }

    Exit timeout = exit::Timeout(params.timeout);
    PID linCont = PID(this -> linConsts);
    PID angCont = PID(this -> angConsts);
    while (timeout.exited() || params.exit.exited({.error = pose.pos.dist(target.pos), .pose = pose })) {
        double h = std::hypot(pose.pos.x - target.pos.x, pose.pos.y - target.pos.y);
        Pt carrot = {target.pos.x - (h * sin(theta) * dLead), target.pos.y - (h * cos(theta) * dLead)};
        dt -> spinVolts(pidMTPVel(target.pos, params, &linCont, &angCont));
    }
}