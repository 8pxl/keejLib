#include "keejLib/control.h"
#include "keejLib/lib.h"

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