#include "main.h"
#include "keejlib/lib.hpp"
#include "pros/motors.hpp"

using namespace keejLib;

std::vector<std::int8_t> DriveTrain::concat(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports) {
    std::vector<std::int8_t> ports(left_ports);
    ports.insert(ports.end(), right_ports.begin(), right_ports.end());
    return ports;
}

void DriveTrain::spinVolts(int left, int right) {
    _motor_group_mutex.take();
    int half = _motor_count / 2;
    for (int i = 0; i < half; i++) {
        _motors[i].move_voltage(left);
        _motors[i + half].move_voltage(right);
    }
    _motor_group_mutex.give();
}

DriveTrain::DriveTrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports) : pros::MotorGroup(concat(left_ports, right_ports)){}