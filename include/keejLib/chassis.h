#pragma once
#include "util.h"

namespace lib
{
    class chassis
    {
        private:
            lib::diffy* chass;
            pros::Imu* imu;
            pros::ADIEncoder* horizTracker = nullptr;
            pros::ADIEncoder* vertTracker = nullptr;
            lib::coordinate pos = {0,0};
            lib::robotConstants constants;
            pros::Task* odomTask = nullptr;
            double prevRotation = 0;

        public:
            chassis(lib::diffy& mtrs, pros::Imu& imu) : chass(&mtrs), imu(&imu){}
            chassis(lib::diffy& mtrs, pros::Imu& imu, lib::robotConstants constants) : chass(&mtrs), imu(&imu), constants(constants) {}
            chassis(lib::diffy& mtrs, pros::Imu& imu, std::vector<int> encoderPorts, lib::robotConstants constants);
            
            void updatePos();
            void initTracking();
            void drive(double target, double timeout, lib::pidConstants constants);
    };
}