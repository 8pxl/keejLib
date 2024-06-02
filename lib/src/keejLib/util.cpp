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

double degToRad(double deg) {
    return deg * M_PI / 180;
}

double radToDeg(double rad) {
    return rad * 180 / M_PI;
}

