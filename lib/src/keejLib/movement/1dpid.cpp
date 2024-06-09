#include "keejLib/lib.h"
#include "keejLib/util.h"
using namespace keejLib;

void Chassis::turn(double angle, MotionParams params = {.async = false, .timeout = 2000, .vMin = 0, .exit = exit::Range(0.01, 500)}) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { turn(angle, params);});
        pros::delay(10);
        return;
    }
    Angle targ = Angle(angle, HEADING);
    Exit timeout = exit::Timeout(params.timeout);
    PID cont = PID(this -> angConsts);
    double error;
    while (params.exit.exited({.error = error}) || timeout.exited()) {
        error = targ.error(Angle(imu -> get_rotation(), HEADING));
        double vel = cont.out(error);
        this -> dt -> spinVolts(vel, -vel);
        pros::delay(10);
    }
}