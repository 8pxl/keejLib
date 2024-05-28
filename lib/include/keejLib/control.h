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
                Stopwatch sw;
                int timeout;
            public:
                Timeout(int timeout);
                bool exit();
        };
        
        class Range : Exit {
            private:
                double range;
                int timeout;
                Stopwatch sw;
            public:
                Range(double range, int timeout);
                bool exit(double error);
        };
    }
    
    struct PIDConstants {
        double kp, ki, kd, kf, maxIntegral, tolerance, integralThreshold;
    };
     
    class PID {
        private:
            double prevError, error, derivative, integral;
            PIDConstants constants;
        public:
            PID(){}
            PID(PIDConstants constants);
            double out(double error);
            double getError();
            double getDerivative();
    };
    
    class VelocityController {
        private:
            double target;
            PID pid;
            EMA ema;
        public:
            VelocityController(PIDConstants cons, double ka);
            void setVelocity(double v);
            double getVoltage(double curr);
    };
}