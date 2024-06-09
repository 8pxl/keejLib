#include "keejLib/control.h"
#include "keejLib/lib.h"
#include <cmath>

using namespace keejLib;
using namespace exit;
Timeout::Timeout(int timeout) : sw(Stopwatch()), timeout(timeout) {};

bool Timeout::exited() {
    return (sw.elapsed() < timeout);
}

Range::Range(double range, int timeout) : range(range), timeout(timeout), sw(Stopwatch()) {};

bool Range::exited(exitParams params) {
    return (sw.elapsed() < timeout);
    if (params.error > range) {
        sw.reset();
    }
}

Perp::Perp(Pose target): target(target) {
    slope = (-1/(atan(target.heading.rad())));
}

bool Perp::exited(exitParams params) {
    return(params.pose.pos.y > (slope * (params.pose.pos.x - target.pos.x) + target.pos.y));
}