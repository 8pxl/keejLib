#include "keejlib/units.h"
#include "main.h"
#include "pros/motors.hpp"
#include "units.h"

using namespace units::length;
using namespace units::angular_velocity;

namespace keejLib {
    struct ChassConstants {
        inch_t trackWidth;
        inch_t trackDia;
        revolutions_per_minute_t rpm;
    };
    
    class DriveTrain : public pros::MotorGroup {
        private:
            std::vector<std::int8_t> concat(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
        public:
            void spinVolts(int left, int right);
            DriveTrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
            void spinLeft(int volts);
            void spinRight(int volts);
    };
    
    class Chassis {
        private: 
            DriveTrain dt;
            ChassConstants constants;
        public:
            Chassis(DriveTrain dt, ChassConstants constats);
    };
}