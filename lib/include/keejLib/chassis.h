#pragma once
#include "main.h"
#include "pros/rotation.hpp"
#include "util.h"
#include "control.h"

namespace keejLib {
    struct ChassConstants {
        double horizWidth;
        double vertWidth;
        double trackDia;
        double wheelDia;
        double gearRatio;
    };
    
    class DriveTrain : public pros::MotorGroup {
        private:
            std::vector<std::int8_t> concat(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
        public:
            DriveTrain(const std::vector<std::int8_t>& left_ports, const std::vector<std::int8_t>& right_ports);
            
            void spinVolts(int left, int right);
            void spinVolts(std::pair<double, double> volts);
            void spinLeft(int volts);
            void spinRight(int volts);

            double getAvgVelocity();
            double getAvgPosition();
    };
    
    struct PrevOdom {
        double vert = 0;
        double horiz = 0;
        Angle theta = Angle(0, DEG);
    };
    
    struct MotionParams {
        bool async;
        int timeout = 3000;
        double vMin = 0;
        double settleRange;
        double settleTime;
        Exit* exit;
        double mtpRotBias;
        double vStart;
        double rotationCut;
        double drift = 0;
        bool reverse = false;
        double within = 0;
        double slew = 0;
    };

    class Chassis {
        private: 
            DriveTrain *dt;
            pros::Imu *imu;
            pros::Rotation *vertEnc;
            pros::Rotation *horizEnc;
            ChassConstants chassConsts;
            PIDConstants linConsts;
            PIDConstants angConsts;
            
            PIDConstants turnConsts;
            
            PIDConstants mtpLin;
            PIDConstants mtpAng;
            Pose pose;
            pros::Task* odomTask = nullptr;
            PrevOdom prev = {0,0};
            bool moving = false;
            Color clr = red;
        public:
            Chassis(DriveTrain *dt, ChassConstants constants, pros::Imu *imu, pros::Rotation *vertEnc, pros::Rotation *horizEnc);
            void update();
            void startTracking();
            void setConstants(PIDConstants linear, PIDConstants angular);
            void setLin(PIDConstants linear);
            void setAng(PIDConstants angular);
            void setMTP(PIDConstants lin, PIDConstants ang);
            void setTurn(PIDConstants turn);
            void setColor(Color c);
            void waitUntilSettled();
            void setPose(Pose p);
            // void resetOdom();
            
            Pose getPose();
            bool isSettled();
            std::pair<double, double> measureOffsets(int iterations);
            double getTheta();
            std::pair<double, double> pidMTPVel(Pt target, MotionParams params, PID* lCont, PID* rCont, double absDist);
            void turn(double angle, MotionParams params);
            void turnTo(Pt target, MotionParams params);
            void driveAngle(double dist, double angle, MotionParams params, bool absolute = false);
            void mtpose(Pose target, double dLead, MotionParams params);
            void mtpoint(Pt target, MotionParams params);
            void moveWithin(Pt targ, double dist, MotionParams params);
    };
}