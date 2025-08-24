#include "keejLib/control.h"
#include "keejLib/lib.h"
#include "keejLib/util.h"

namespace keejLib {
    
void Chassis::turn(double angle, MotionParams params) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { turn(angle, params);});
        pros::delay(10);
        return;
    }
    this -> waitUntilSettled();
    if (params.exit == nullptr) {
        params.exit = new exit::Range(1.5, 20);
    }
    moving = true;
    // if (clr == blue) angle = neg(angle);
    Angle targ = Angle(angle, HEADING);
    Exit* timeout = new exit::Timeout(params.timeout);
    PID cont = PID(this -> turnConsts);
    double error = targ.error(Angle(imu -> get_heading(), HEADING));

    while (!params.exit -> exited({.error = fabs(error)}) && !timeout -> exited({})) {
        error = targ.error(Angle(imu -> get_heading(), HEADING));
        double vel = cont.out(error);
        vel = std::clamp(vel, -params.vMax, params.vMax);
        this -> dt -> spinVolts({vel, -vel});
        pros::delay(10);
    }
    this -> dt -> spinAll(0);
    moving = false;
}

double Chassis::turnTo(Pt target, MotionParams params) {
    // chassMutex.take();
    if (params.async) {
        params.async = false;
        double returnVal;
        pros::Task task([&]() {returnVal = turnTo(target, params);});
        pros::delay(10);
        return returnVal;
    }
    this -> waitUntilSettled();
    if (params.exit == nullptr) {
        params.exit = new exit::Range(1.5, 20);
    }
    moving = true;
    if (clr == blue) target = translate(target);
    Angle targ = absoluteAngleToPoint(pose.pos, target);
    if (params.reverse) targ = targ.reverseDir();
    Exit* timeout = new exit::Timeout(params.timeout);
    // Exit* range = new exit::Range(params.settleRange, params.settleTime);
    PID cont = PID(this -> turnConsts);
    double error = targ.error(Angle(imu -> get_heading(), HEADING));
    while (!params.exit -> exited({.error = fabs(error)}) && !timeout -> exited({})) {
        targ = absoluteAngleToPoint(pose.pos, target);
        if (params.reverse) targ = targ.reverseDir();
        error = targ.error(Angle(imu -> get_heading(), HEADING));
        double vel = cont.out(error);
        vel = sign(vel) * std::min(fabs(vel), params.vMax);
        this -> dt -> spinVolts({vel, -vel});
        // std::cout << "turnto: " << error << std::endl;
        pros::delay(10);
    }
    this -> dt -> spinAll(0);
    moving = false;
    return targ.heading();
    // chassMutex.give();
}

void Chassis::swingTo(Pt target, double radius, MotionParams params) {
    if (params.async) {
        params.async = false;
        double returnVal;
        pros::Task task([&]() {returnVal = turnTo(target, params);});
        pros::delay(10);
    }
    this -> waitUntilSettled();
    if (params.exit == nullptr) {
        params.exit = new exit::Range(1.5, 20);
    }
    moving = true;
    Angle targ = absoluteAngleToPoint(pose.pos, target);
    if (params.reverse) targ = targ.reverseDir();
    Exit* timeout = new exit::Timeout(params.timeout);
    PID cont = PID(this -> linConsts);
    double error = targ.error(Angle(imu -> get_heading(), HEADING));
    double ratio = fabs((fabs(radius) + this ->chassConsts.vertWidth) / (fabs(radius) - this ->chassConsts.vertWidth));
    while (!params.exit -> exited({.error = fabs(error)}) && !timeout -> exited({})) {
        targ = absoluteAngleToPoint(pose.pos, target);
        if (params.reverse) targ = targ.reverseDir();
        error = targ.error(Angle(imu -> get_heading(), HEADING));
        double vel = cont.out(error);
        vel = sign(vel) * std::clamp(fabs(vel), -params.vMax, params.vMax);
        if (std::abs(vel) < params.vMin && params.vMin != 0) {
            vel = params.vMin * sign(vel);
        }
        double rvel = (2 * vel) / (ratio+1);
        rvel = std::abs(rvel) >= 127 ? (127 * sign(rvel)) : rvel;
        double lvel = ratio * rvel;
        if (radius < 0) this -> dt -> spinVolts({rvel, lvel});
        else this -> dt -> spinVolts({-lvel, -rvel});
        // std::cout << "turnto: " << error << std::endl;
        pros::delay(10);
    }
    this -> dt -> spinAll(0);
    moving = false;
}
// void Chassis::linTo(Pt target, MotionParams params) {
//     if (params.async) {
//         params.async = false;
//         pros::Task task([&]() { linTo(target, params);});
//         pros::delay(10);
//         return;
//     }
//     this -> waitUntilSettled();
//     moving = true;

//     heading = Angle(angle, HEADING);
// }
}