#pragma once
#include "main.h"
#include "units.h"
#include "util.h"

using namespace units::length;
using namespace units::angular_velocity;

namespace keejLib {
    struct ChassConstants {
        inch_t trackWidth;
        inch_t trackDia;
        inch_t wheelDia;
        double gearRatio;
    };
    
    class DriveTrain : public pros::MotorGroup {
        private:
            std::vector<std::int8_t> concat(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
        public:
            DriveTrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
            void spinVolts(int left, int right);
            void spinLeft(int volts);
            void spinRight(int volts);
    };
    
    class Chassis {
        private: 
            DriveTrain dt;
            ChassConstants constants;
            pt pos;
            pros::Task* odomTask = nullptr;
        public:
            Chassis(DriveTrain dt, ChassConstants constats);
            void updatePos();
            void startTracking();
    };
}