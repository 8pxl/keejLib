#pragma once

#include <cmath>
#include <vector>
#include "main.h"

#define PI 3.14159265358979323846
#define ALLBUTTONS {pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A}
typedef void(*fptr)();

namespace lib
{
    enum buttons
    {
        L1 = 0,
        NL1 = 1,
        L2 = 2,
        NL2 = 3,
        R1 = 4,
        NR1 = 5,
        R2 = 6,
        NR2 = 7,
        UP = 8,
        NUP = 9,
        DOWN = 10,
        NDOWN = 11,
        LEFT = 12,
        NLEFT = 13,
        RIGHT = 14,
        NRIGHT = 15,
        X = 16,
        NX = 17,
        B = 18,
        NB = 19,
        Y = 20,
        NY = 21,
        A = 22,
        NA = 23 
    };

    struct pidConstants{double p,i,d,tolerance,integralThreshold, maxIntegral;};

    struct coordinate{double x,y;};

    struct robotConstants{double horizTrack, vertTrack, trackDia, maxSpeed, fwdAccel, fwdDecel, revAccel, revDecel, ip10mstomvolt;};
    //inches per 10 ms to motor volt
    struct atns{std::vector<fptr> autonsList; std::vector<std::string> names; };


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

            double out(double error);
    };

    double dtr(double input);
    double rtd(double input);
    int dirToSpin(double target,double currHeading);
    double minError(double target, double current);;
    double imuToRad(double heading);
    double sign(double a);
    double hypot(double a, double b);
    double dist(coordinate a, coordinate b);
    double absoluteAngleToPoint(lib::coordinate pos, lib::coordinate point);
}