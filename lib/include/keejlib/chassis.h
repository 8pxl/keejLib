#include "main.h"
#include "pros/motors.hpp"
#include "units.h"

namespace keejLib {
    class DriveTrain : pros::MotorGroup {
        private:
            std::vector<std::int8_t> concat(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
        public:
            void spinVolts(int left, int right);
            DriveTrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
    };
    
    class Chassis {
        public:
            
    };
}