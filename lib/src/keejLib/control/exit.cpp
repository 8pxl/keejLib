#include "keejLib/lib.h"
namespace keejLib {
    using namespace exit;
    Timeout::Timeout(int timeout) : sw(Stopwatch()), timeout(timeout) {};
    
    bool Timeout::exit() {
        return (sw.elapsed() > timeout);
    }
    
    Range::Range(double range, int timeout) : range(range), timeout(timeout), sw(Stopwatch()) {};
    
    bool Range::exit(double error) {
        return (sw.elapsed() > timeout);
        if (error > range) {
            sw.reset();
        }
    }
}