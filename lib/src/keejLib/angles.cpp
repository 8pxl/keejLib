#pragma once

#include "keejLib/lib.h"
#include "util.h"

double keejLib::toRad(double deg) {
    return deg * M_PI / 180;
}

double keejLib::toDeg(double rad) {
    return rad * 180 / M_PI;
}

int keejLib::dirToSpin(double target, double current) {
    double d = (target - current);
    double diff = d < 0 ? d + 360 : d;
    return (diff > 180 ? -1 : 1);
}

double keejLib::angError(double target, double current) {
    double b = std::fmax(target, current);
    double s = std::fmin(target, current);
    double diff = b - s;
    
    return((diff <= 180 ? diff : (360-b) + s) * keejLib::dirToSpin(target, current));
}

double keejLib::toStandard(double deg) {
    double a = 90 - deg;
    a = a < 0 ? 360 + a : a;
    return (keejLib::toRad(a));
}

double keejLib::fromStandard(double rad) {
    double a = M_PI/2 - rad;
    a = a >= 0 ? a : 2 * M_PI + a;
    return (keejLib::toDeg(a));
}

double keejLib::reverseDir(double heading) {
    return(fmod(heading + 180, 360));
}

double keejLib::mtpAngleError(const Pose& pose, const Pt& target, int dir) {
    Angle currHeading = pose.heading;
    Angle targetHeading = absoluteAngleToPoint(pose.pos, target);
    if (dir < 0) targetHeading = Angle(reverseDir(targetHeading.heading()), HEADING);
    return targetHeading.error(currHeading);
}

keejLib::Angle::Angle() {
    this -> angle_s = 0;
}
keejLib::Angle::Angle(double angle, AngleType type) {
    switch (type) {
        case AngleType::DEG:
            this -> angle_s = toRad(angle);
            break;
        case AngleType::RAD:
            this -> angle_s = angle;
            break;
        case AngleType::HEADING:
            this -> angle_s = toStandard(angle);
    }
}

keejLib::Angle keejLib::Angle::operator+(Angle& other) {
    return Angle(this -> angle_s + other.angle_s, AngleType::RAD);
}

void keejLib::Angle::operator+=(Angle& other) {
    this -> angle_s += other.angle_s;
}

keejLib::Angle keejLib::Angle::operator-(Angle& other) {
    return Angle(this -> angle_s - other.angle_s, AngleType::RAD);
}

keejLib::Angle keejLib::Angle::operator/(double b) {
    return Angle(this -> angle_s / b, AngleType::RAD);
}

bool keejLib::Angle::operator==(double b) {
    return this -> angle_s == b;
}

double keejLib::Angle::rad() {
    return angle_s;
}

double keejLib::Angle::deg() {
    return angle_s * 180 / M_PI;
}

double keejLib::Angle::heading() {
   return fromStandard(angle_s);
}

double keejLib::Angle::error(Angle current) {
    return angError(heading(), current.heading());
} 

keejLib::Angle keejLib::Angle::reverseDir() {
    return Angle(keejLib::reverseDir(this->heading()), HEADING);
}