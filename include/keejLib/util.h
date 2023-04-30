#pragma once

#include <cmath>
#include <vector>
#include "main.h"
#define PI 3.14159265358979323846

namespace lib
{
    enum buttons;

    struct pidConstants{double p,i,d,tolerance,integralThreshold, maxIntegral;};

    struct coordinate{double x,y;};

    struct robotConstants{double horizTrack, vertTrack, trackDia, maxSpeed, fwdAccel, fwdDecel, revAccel, revDecel;};

    class timer
    {
        public:
            int startTime = 0;
            timer();
            void start();
            int time();
    };

    class pid
    {
        private:
            double prevError,error,derivative,integralThreshold;
            double integral = 0;
            pidConstants constants;

        public:
            pid(){}

            pid(pidConstants cons, double error) : constants(cons), prevError(error){}

            double out(double error){}
    };

    class controller
    {
        private:
            pros::Controller* cont;
            double leftCurve, rightCurve;

        public:
            controller(pros::Controller& cont) : cont(&cont) {}

            enum driveMode{};

            int select(int num, std::vector<std::string> names);
            
            std::vector<bool> getAll(std::vector<pros::controller_digital_e_t> buttons);

            double curve(double x, double scale);

            std::vector<double> drive(int direction, controller::driveMode mode);

            void setCurves(double left, double right);
    };

    double dtr(double input);
    double rtd(double input);
    int dirToSpin(double target,double currHeading);
    double minError(double target, double current);;
    double imuToRad(double heading);
    double sign(double a);
    double hypot(double a, double b);
}