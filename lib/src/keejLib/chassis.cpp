#include "chassis.h"
#include "main.h"
#include "keejlib/lib.h"
#include "util.h"
#include <numeric>

namespace keejLib {
    
DriveTrain::DriveTrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports) : pros::MotorGroup(concat(left_ports, right_ports)){}

std::vector<std::int8_t> DriveTrain::concat(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports) {
    std::vector<std::int8_t> ports(left_ports);
    ports.insert(ports.end(), right_ports.begin(), right_ports.end());
    return ports;
}

void DriveTrain::spinVolts(int left, int right) {
    _motor_group_mutex.take();
    int half = _motor_count / 2;
    for (int i = 0; i < half; i++) {
        _motors[i].move(left);
        _motors[i + half].move(right);
    }
    _motor_group_mutex.give();
}

void DriveTrain::spinVolts(std::pair<double, double> volts) {
    spinVolts(volts.first, volts.second);
}
double DriveTrain::getAvgVelocity() {
    std::vector<double> v = (get_actual_velocities());
    return (std::reduce(v.begin(), v.end()) / v.size());
}

double DriveTrain::getAvgPosition() {
    std::vector<double> v = (get_positions());
    return (std::reduce(v.begin(), v.end()) / v.size());
}

Chassis::Chassis(keejLib::DriveTrain *dt, keejLib::ChassConstants constants, pros::Imu *imu, pros::Rotation *vertEnc, pros::Rotation *horizEnc) : dt(dt), chassConsts(constants), imu(imu), vertEnc(vertEnc), horizEnc(horizEnc) {}

std::pair<double, double> Chassis::measureOffsets(int iterations) {
    std::pair<double, double> offsets = {0,0};
    for (int i = 0; i < iterations; i++) {
        std::pair<double, double> deltaEnc = {0, 0};
        imu -> reset(true);
        double imuStart = imu -> get_heading();
        double target = i%2 == 0 ? 90 : 270;
        this -> turn(target, {.async = true, .timeout=1000, .exit = new exit::Range(0.01, 500)});
        Stopwatch s;
        PrevOdom prev = {0,0};
        vertEnc -> reset_position();
        horizEnc -> reset_position();
        while (s.elapsed() < 1000) {
            double currVert = vertEnc -> get_position() / 100.0;
            double currHoriz = horizEnc -> get_position()/ 100.0;
            
            deltaEnc.first += fabs(angError(currVert, prev.vert));
            deltaEnc.second += fabs(angError(currHoriz, prev.horiz));
            std::cout << "vert: " << deltaEnc.first << " horiz: " << deltaEnc.second << std::endl;
            prev.vert = currVert;
            prev.horiz = currHoriz;
            pros::delay(10);
        }
        double delta = toRad(fabs(angError(imu -> get_heading(), imuStart)));
        std::cout << delta << std::endl;
        offsets.first += ((deltaEnc.first * M_PI * chassConsts.wheelDia) / 360) / delta;
        offsets.second += ((deltaEnc.second * M_PI * chassConsts.wheelDia) / 360) / delta;
    }
    return {offsets.first / iterations, offsets.second / iterations};
}

void Chassis::setLin(PIDConstants linear) {
    linConsts = linear;
}

void Chassis::setAng(PIDConstants ang) {
    angConsts = ang;
}

void Chassis::setMTP(PIDConstants lin, PIDConstants ang) {
    mtpLin = lin;
    mtpAng = ang;
}

void Chassis::setTurn(PIDConstants turn) {
    turnConsts = turn;
}

void Chassis::waitUntilSettled() {
    while (moving) {
        pros::delay(10);
    }
}

bool Chassis::isSettled() {
    return !moving;
}

Pose Chassis::getPose() {
    return pose;
}

void Chassis::setColor(Color c) {
    clr = c;
}

void Chassis::setPose(Pose p) {
    pose = p;
}
}