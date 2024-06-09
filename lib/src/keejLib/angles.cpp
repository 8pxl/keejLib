#include "keejLib/lib.h"
#include "keejLib/util.h"

using namespace keejLib;

double toRad(double deg) {
    return deg * M_PI / 180;
}

double toDeg(double rad) {
    return rad * 180 / M_PI;
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

double toStandard(double deg) {
    double a = 90 - deg;
    a = a < 0 ? 360 + a : a;
    return (keejLib::toRad(a));
}

double fromStandard(double rad) {
    double a = M_PI/2 - rad;
    a = a > 0 ? a : 2 * M_PI + a;
    return (keejLib::toDeg(a));
}

Angle::Angle(double angle, AngleType type) {
    switch (type) {
        case AngleType::DEG:
            this -> angle_s = toRad(angle);
            break;
        case AngleType::RAD:
            this -> angle_s = angle;
            break;
        case AngleType::HEADING:
            this -> angle_s = fromStandard(angle);
    }
}

Angle Angle::operator+(Angle& other) {
    return Angle(this -> angle_s + other.angle_s, AngleType::RAD);
}

void Angle::operator+=(Angle& other) {
    this -> angle_s += other.angle_s;
}

Angle Angle::operator-(Angle& other) {
    return Angle(this -> angle_s - other.angle_s, AngleType::RAD);
}

Angle Angle::operator/(double b) {
    return Angle(this -> angle_s / b, AngleType::RAD);
}

bool Angle::operator==(double b) {
    return this -> angle_s == b;
}

double Angle::rad() {
    return angle_s;
}

double Angle::deg() {
    return angle_s * 180 / M_PI;
}

double Angle::heading() {
   return toStandard(angle_s);
}

double Angle::error(Angle current) {
    return angError(heading(), current.heading());
} 