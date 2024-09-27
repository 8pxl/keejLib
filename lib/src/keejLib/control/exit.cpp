#include "keejLib/control.h"
#include "keejLib/lib.h"
#include <cmath>

namespace keejLib {
exit::Timeout::Timeout(int timeout) : sw(Stopwatch()), timeout(timeout) {};

bool exit::Timeout::exited(exitParams params) {
    return (sw.elapsed() > timeout);
}

exit::Range::Range(double range, int timeout) : range(range), timeout(timeout), sw(Stopwatch()) {};

bool exit::Range::exited(exitParams params) {
    if (params.error > range) {
        sw.reset();
    }
    return (sw.elapsed() > timeout);
}

exit::Perp::Perp(Pose target): target(target) {
    slope = (-1/(atan(target.heading.rad())));
}

bool exit::Perp::exited(exitParams params) {
    return(params.pose.pos.y > (slope * (params.pose.pos.x - target.pos.x) + target.pos.y));
}
}