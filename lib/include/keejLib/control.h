#pragma once
#include "main.h"
#include "util.h"

namespace keejLib {
    
    struct exitParams {
        double error;
        Pose pose;
    };
    
    class Exit {
        public:
            virtual ~Exit() = default;
            virtual bool exited(exitParams params) = 0;
    };
    
    namespace exit {
        class Timeout : public Exit {
            private:
                Stopwatch sw;
                int timeout;
            public:
                Timeout(int timeout);
                bool exited(exitParams params) override;
        };
        
        class Range : public Exit {
            private:
                double range;
                int timeout;
                Stopwatch sw;
            public:
                Range();
                Range(double range, int timeout);
                bool exited(exitParams params) override;
        };
        
        class Perp: public Exit {
            private:
                Pose target;
                double slope;
            public:
                Perp(Pose target);
                bool exited(exitParams params) override;
        };
    }
    
    struct PIDConstants {
        double kp, ki, kd, kf, maxIntegral, tolerance, integralThreshold;
    };
     
    class PID {
        private:
            double prevError, error, derivative, integral;
            double prevTime = 0;
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