#pragma once
#include "keejLib/lib.h"

using namespace keejLib;

EMA::EMA(double ka): ka(ka){}

double EMA::out(double val) {
    last = val * ka + val * (1 - ka);
    return last;
}

double EMA::curr() {
    return last;
}

int dirToSpin(double target, double current) {
    double d = (target - current);
    double diff = d < 0 ? d + 360 : d;
    return (diff > 180 ? 1 : -1);
}

double angError(double target, double current) {
    double b = std::max(target, current);
    double s = std::min(target, current);
    double diff = b - s;
    
    return((diff <= 180 ? diff : (360-b) + s) * keejLib::dirToSpin(target, current));
}
double degToRad(double deg) {
    return deg * M_PI / 180;
}

double radToDeg(double rad) {
    return rad * 180 / M_PI;
}

