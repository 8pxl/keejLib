#pragma once
#include "main.h"
#include "pros/distance.hpp"
#include "util.h"

namespace keejLib {
    
    struct exitParams {
        double error;
        Pose pose;
        Pt target;
        Angle targetHeading;
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
                std::optional<Pt> target;
                std::optional<Angle> targetHeading;
                std::optional<int> side;
            public:
                Perp();
                Perp(Pt target);
                Perp(Pt target, Angle targetHeading);
                int computeSide(exitParams params);
                bool exited(exitParams params) override;
        };
        
        class Within: public Exit {
            private:
                Pt target;
                double range;
                int timeout;
                Stopwatch sw;
            public:
                Within(Pt target, double range, int timeout);
                bool exited(exitParams params) override;
        };
        
        class DistanceSensor: public Exit {
            private:
                pros::Distance *distance;
                double threshold;
                int delay;
                int startDelay;
                Timer timer;
                Timer startTimer;
            public: 
                DistanceSensor(pros::Distance *distance, double threshold, int startDelay, int delay);
                bool exited(exitParams params) override;
        };
    }
    
    struct PIDConstants {
        double kp, ki, kd, kf, maxIntegral, tolerance, integralThreshold;
    };
    
    //all units are percentage values (from -1 to 1), this should be exposed to the user (units handled in the background)
    struct ProfileParams {
        double speedPct, accelPct, decelPct = 1;
        double startPct = 0;
        double endPct = 0;
    };
    
    struct ProfileLimits {
        double maxSpeed, maxAccel, maxDecel;
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
            void resetIntegral();
    };
    
    std::vector<double> generateProfile(double dist, ProfileParams pp, ProfileLimits pl);
    
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
    
    class VelocityManager {
        private: 
            double vl, va, linMin, linMax, angMin, angMax;
            std::optional<double> slew;
        public:
            VelocityManager(
                double vl = 0, 
                double va = 0, 
                double s = 0,
                double linMin = 0, 
                double linMax = 127,
                double angMin = 0, 
                double angMax = 127)
                : vl(vl), va(va), linMin(linMin), linMax(linMax), angMin(angMin), angMax(angMax) {
                    if (s != 0) slew = s;
                }
            ChassVelocities update(std::pair<double, double> vals);
            
            void setLinMin(double newLinMin);
            void setLinMax(double newLinMax);
            void setAngMin(double newAngMin);
            void setAngMax(double newAngMax);
            void setSlew(double newSlew);
    };
}