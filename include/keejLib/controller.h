#pragma once
#include "main.h"

namespace lib
{
    class controller
    {
        private:
            pros::Controller* cont;
            double leftCurve;
            double rightCurve;

        public:
            controller(pros::Controller& cont) : cont(&cont) {}

            enum driveMode
            {
                arcade,
                tank
            };

            int select(int num, std::vector<std::string> names);
            
            std::vector<bool> getAll(std::vector<pros::controller_digital_e_t> buttons);

            double curve(double x, double scale);

            std::vector<double> drive(int direction, controller::driveMode mode);

            void setCurves(double left, double right);
    };
}