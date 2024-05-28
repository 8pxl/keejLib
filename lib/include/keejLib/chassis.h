#pragma once
#include "main.h"
#include "util.h"
#include "control.h"

namespace keejLib {
    struct ChassConstants {
        double trackWidth;
        double trackDia;
        double wheelDia;
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

            double getAvgVelocity();
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
            
            void mtp(double x, double y, double theta, double dLead, double vMin, Exit exit);
    };
}