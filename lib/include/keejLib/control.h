#pragma once
#include "main.h"
#include "util.h"

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
                Timeout(int timeout);
                bool exit();
        };
        
        class Range : Exit {
            private:
                double range;
                Stopwatch* sw;
            public:
                Range(double range, int timeout);
                bool exit(double val);
        };
    }
    
    struct PIDConstants {
        double kp, ki, kd, kf, maxIntegral, tolerance, integralThreshold;
    };
     
    class PID {
        private:
            E_unit prevError, error;
            double derivative, integral;
            PIDConstants constants;
        public:
            PID(){}
            PID(PIDConstants constants);
            O_unit out(double error);
            E_unit getError();
    };
    
    class VelocityController {
        private:
            pros::MotorGroup* mtrs;
            revolutions_per_minute_t target;
            PID pid;
            EMA ema;
        public:
            VelocityController(pros::MotorGroup* mtrs, PIDConstants cons, double ka);
            void setVelocity(revolutions_per_minute_t v);
            void applyVoltage();
    };
}