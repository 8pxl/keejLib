#include "main.h"
#include "pros/motors.hpp"
#include <array>
#include "units.h"

// using namespace units::literals;
using namespace units::angular_velocity;

namespace keejLib {
    class DriveTrain : pros::MotorGroup {
        private:
            std::vector<std::int8_t>* concat(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
        public:
            void spinVolts(int left, int right) {
                _motor_group_mutex.take();
                int half = _motor_count / 2;
                for (int i = 0; i < half; i++) {
                    _motors[i].move_voltage(left);
                    _motors[i + half].move_voltage(right);
                }
                _motor_group_mutex.give();
            }
            DriveTrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
    };
    
    class Chassis {
        public:
            
    };
}