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

double pt::dist(pt a) {
    return sqrt(pow(a.x - x, 2) + pow(a.y - y, 2));
}

template <typename T>
int sign(T x) {
    return(x > 0 ? 1 : -1);
}