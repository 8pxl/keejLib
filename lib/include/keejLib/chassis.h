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
            point pos = {0,0};
            lib::robotConstants constants;
            lib::accelConstants linear;
            lib::accelConstants angular;
            pros::Task* odomTask = nullptr;
            double prevRotation = 0;

        public:
            lib::odomType odom;
            chassis(lib::diffy& mtrs, pros::Imu& imu) : chass(&mtrs), imu(&imu){}
            chassis(lib::diffy& mtrs, pros::Imu& imu, lib::robotConstants constants, lib::accelConstants linear, lib::accelConstants angular) : chass(&mtrs), imu(&imu), constants(constants), linear(linear), angular(angular) {}
            chassis(lib::diffy& mtrs, pros::Imu& imu, std::vector<int> encoderPorts, lib::robotConstants constants, lib::accelConstants linear, lib::accelConstants angular);
            
            void updatePos();
            void initTracking();

            //1dpid
            lib::pid pidDrive(double target, double timeout, lib::pidConstants constants, char brake);
            lib::pid pidTurn(double target, double timeout, lib::pidConstants constants, char brake);
            lib::pid pidDrive(double target, double timeout, char brake, lib::pid cont);
            lib::pid pidTurn(double target, double timeout, char brake, lib::pid cont);
            // lib::pid keejTurn(double target, int timeout, lib::pidConstants constants, char brake);


            void arcTurn(double target, double radius, double timeout, int dir, lib::pidConstants constants, double min, int endTime, char brake); 
            void eulerTurn(double theta, double rate, double timeout, int dir, lib::pidConstants constants);

            //1dmp
            std::vector<double> asymTrapezoidalProfile(double dist, double maxSpeed, double accel, double decel, double start, double end);
            void profiledDrive(double target, int endDelay, double start, double end);
            void profiledTurn(double target, int dir, int endDelay);
            void timedDrive(int time, int speed);


            //2dpid
            void driveAngle(double target, double heading, double timeout, lib::pidConstants lCons, lib::pidConstants acons);
            std::vector<double> pidMTPVel(const point& target, double rotationBias, lib::pid* lCont, lib::pid* rCont);
            void pidMoveTo(const point& target, double timeout, lib::pidConstants lConstants, lib::pidConstants 
            rConstants, double rotationBias);
            void boomerang(const point& target, double timeout, double dLead, double thetaEnd, double rotationBias, lib::pidConstants lConstants, lib::pidConstants rConstants);

            //pp
            lib::point targetPoint(lib::linearPath path, int lookAhead, int lineLookAhead, int lineIndex);
            void moveToPurePursuit(lib::linearPath path, double lookAhead, int lineLookAhead, int finalTimeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias);

    };
}