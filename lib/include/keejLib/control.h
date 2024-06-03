#pragma once
#include "main.h"
#include "util.h"

namespace keejLib {
    
    struct exitParams {
        double error;
    };
    
    class Exit {
        public:
            virtual bool exited(exitParams);
            virtual bool exited();
    };
    
    namespace exit {
        class Timeout : public Exit {
            private:
                Stopwatch sw;
                int timeout;
            public:
                Timeout(int timeout);
                bool exited();
        };
        
        class Range : public Exit {
            private:
                double range;
                int timeout;
                Stopwatch sw;
            public:
                Range();
                Range(double range, int timeout);
                bool exited(exitParams params);
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