#pragma once
#include "main.h"
#include "util.h"
#include "pros/motors.hpp"
#include "units.h"


using namespace units;
using namespace angular_velocity;

namespace keejLib {
    class Exit {
        public:
            virtual bool exit();
    };
    
    namespace exit {
        class Timeout : Exit {
            private:
                Stopwatch* sw;
            public:
                Timeout(millisecond_t* timeout);
                bool exit();
        };
        
        class Range : Exit {
            private:
                double range;
                Stopwatch* sw;
            public:
                Range(double range, millisecond_t timeout);
                bool exit(double val);
        };
    }
    
    struct PidConstants {
        double p,i,d,f,integralThreshold, tolerance, maxIntegral;
    };
    
    
    class Pid {
        private:
            double prevError, error, derivative, integral;
            PidConstants constants;
        public:
            Pid(){}
            Pid(PidConstants constants);
            double out(double error);
            double getError();
            double getDerivative();
    };
    
    class VelocityController {
        private:
            pros::MotorGroup* mtrs;
            revolutions_per_minute_t target;
            Pid controller;
        public:
            VelocityController(pros::MotorGroup* mtrs, PidConstants constants);
            void setVelocity(revolutions_per_minute_t v);
            void applyVoltage();
    };
}