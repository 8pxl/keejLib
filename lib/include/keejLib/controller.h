#pragma once
#include "main.h"

namespace keejLib {
    class Controller {
        private:
            pros::Controller* cont;
            double leftCurve;
            double rightCurve;

        public:
            Controller(pros::Controller& cont);

            enum driveMode{
                arcade,
                tank,
                reverseArcade,
                curvature
            };

            int select(std::vector<std::string> names);
            std::vector<bool> getAll(std::vector<pros::controller_digital_e_t> buttons);
            double curve(double x, double scale);
            std::pair<double, double> drive(int direction, Controller::driveMode mode);
            void setCurves(double left, double right);
    };
    
    // std::vector<Controller::driveMode> driveModes = {keejLib::Controller::driveMode::tank, keejLib::Controller::driveMode::arcade, keejLib::Controller::driveMode::curvature};
    #define DRIVEMODE_NAMES {"arcade", "tank", "reverse arcade", "curvature"}
}