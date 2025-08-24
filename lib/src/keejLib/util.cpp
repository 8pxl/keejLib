#include "keejLib/lib.h"
#include "pros/rtos.hpp"


namespace keejLib {

EMA::EMA(double ka): ka(ka){}

double EMA::out(double val) {
    last = val * ka + last * (1 - ka);
    return last;
}

double EMA::curr() {
    return last;
}

double Pt::dist(Pt a) {
    return sqrt(pow(a.x - x, 2) + pow(a.y - y, 2));
}

Angle absoluteAngleToPoint(const Pt &pos, const Pt &point) {
    double t;
    try { 
        t = atan2(point.x - pos.x, point.y - pos.y);
    }

    catch(...) {
        t = M_PI/2;
    }
    
    t = toDeg(t);
    t = t >= 0 ? t : 360+t;
    return (Angle(t, HEADING));
}

Color oppositeColor(Color c) {
    if (c == blue) return red;
    if (c == red) return blue;
    return none;
}
Stopwatch::Stopwatch() {
    start = pros::millis();
}

void Stopwatch::reset() {
    start = pros::millis();
}

int Stopwatch::elapsed() {
    return pros::millis() - start;
}

Timer::Timer(int timeout) {
    set(timeout);
}

void Timer::reset() {
    startTime = pros::millis();
}

void Timer::set(int timeout) {
    this->timeout = timeout;
}

void Timer::start() {
    if (live) return;
    startTime = pros::millis();
    live = true;
}
void Timer::stop() {
    live = false;
}

bool Timer::done() {
    if (!live) return false;
    return (pros::millis() - startTime) > timeout;
}
//credit: lemlib
double curvature(Pose pose, Pose other) {
    // calculate whether the pose is on the left or right side of the circle
    double heading = pose.heading.rad();
    // if (heading > 180) heading = - (360 - heading);
    // heading = toRad(heading);

    float side = keejLib::sign(std::sin(heading) * (other.pos.x - pose.pos.x) - std::cos(heading) * (other.pos.y - pose.pos.y));
    // calculate center point and radius
    float a = -std::tan(heading);
    float c = std::tan(heading) * pose.pos.x - pose.pos.y;
    float x = std::fabs(a * other.pos.x + other.pos.y + c) / std::sqrt((a * a) + 1);
    float d = std::hypot(other.pos.x - pose.pos.x, other.pos.y - pose.pos.y);

    // return curvature
    return side * ((2 * x) / (d * d));
}

//https://www.desmos.com/calculator/fxvpbuowqe
Pt triangulate(Pose a, Pose b) {
    double tt1 = tan(a.heading.rad());
    double tt2 = tan(b.heading.rad());
    double x = (a.pos.x*tt1 - b.pos.x*tt2 + b.pos.y - a.pos.y) / (tt1 - tt2);
    double y = tt1 * (x- a.pos.x) + b.pos.y;
    
    return {x,y};
}

Pt translate(Pt a) {
    return {-a.x, a.y};
}

double calculateMaxSlipSpeed(const Pose& pose, const Pt& target, double drift) {
    double radius = 1 / fabs(curvature(pose, {target, Angle(0, RAD)}));
    if (drift == 0) {
        return 127;
    } else {
        return sqrt(drift * radius * 9.8);
    }
}

int computeSide(const Pt& curr, const Pt& target, Angle heading) {
    double adjHeading = heading.rad();
    if (adjHeading > M_PI) {
        adjHeading = -(2 * M_PI - adjHeading);
    }
    
    double m = tan(adjHeading);
    
    int side = (curr.y < (-1 / m) * (curr.x - target.x) + target.y) ? 1 : -1;
    
    if (adjHeading < 0) {
        side = -side;
    }
    
    return side;
}

}