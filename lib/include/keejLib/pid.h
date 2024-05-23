#pragma once
#include "main.h"
#include "util.h"
#include "pros/motors.hpp"
#include "units.h"


using namespace units;
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
        double p,i,d,f,integralThreshold, maxIntegral;
    };
    
    
    class Pid {
        private:
            double prevError, error, derivative, integral;
        public:
            Pid(){}
            double out(double error);
            double getError();
            double getDerivaative();
    };
}