#pragma once
#include "main.h"

namespace lib
{
    class mtrs
    {
        private:
            pros::motor_brake_mode_e returnBrakeType(char brakeMode);

        protected:
            std::vector<pros::Motor> motors;
            int size;

        public:
            mtrs(const std::vector<int> & ports);

            void spin(double volts);
            void stop(char brakeMode);
            void setBrake(char brakeMode);
            void reset();
            double getSpeed();
            double getRotation();
    };

    class diffy : public mtrs
    {
        public:
            using mtrs::mtrs;

            void spinDiffy(double rvolt, double lvolt);
            void spinDiffy(std::vector<double> voltages);
            std::vector<double> getDiffy();
    };

    class pis
    {
        private:
            std::vector<pros::ADIDigitalOut> pistons;
            bool state;
        
        public:
            pis(std::vector<pros::ADIDigitalOut> p, bool s);

            void toggle();
            void setState(bool iState);
            bool getState();
    };
}