#pragma once
#include "main.h"
#include "pros/rotation.hpp"
#include "util.h"
#include "control.h"

namespace keejLib {
    struct ChassConstants {
        double horizWidth;
        double vertWidth;
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
    
    struct prevOdom {
        double vert;
        double horiz;
        double theta;
    };
    
    class Chassis {
        private: 
            DriveTrain *dt;
            pros::Imu *imu;
            pros::Rotation *vertEnc;
            pros::Rotation *horizEnc;
            ChassConstants constants;
            PIDConstants linear;
            PIDConstants angular;
            Pose pose;
            pros::Task* odomTask = nullptr;
            prevOdom prev;
        public:
            Chassis(DriveTrain *dt, ChassConstants constants);
            void update();
            void startTracking();
            void setConstants(PIDConstants linear, PIDConstants angular);
            void setLin(PIDConstants linear);
            void setAng(PIDConstants angular);
            
            void driveAngle(double dist, double angle, bool async, double vMin, Exit exit);
            // void mtp(double x, double y, double theta, double dLead, double vMin, Exit exit);
    };
}