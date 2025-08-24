#include "keejLib/lib.h"
#include "keejLib/util.h"
#include "main.h"

keejLib::Controller::Controller(pros::Controller& cont) : cont(&cont) {}

int keejLib::Controller::select(std::vector<std::string> names) {
    int num = names.size();
    int curr = 0;
    cont -> clear();
    while(1) {
        if(cont -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (curr != num-1) {
                curr++;
            }

            else {
                curr = 0;
            }
        }

        if(cont -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            if (curr != 0) {
                curr--;
            }

            else{
                curr = num-1;
            }
        }

        if(cont -> get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::delay(200);
            return(curr);
        }

        cont -> print(0, 0, "%s         ", names[curr]);
        pros::delay(50);
    }
}

std::vector<bool> keejLib::Controller::getAll(std::vector<pros::controller_digital_e_t> buttons) {
    std::vector<bool> out;
    for (pros::controller_digital_e_t button : buttons) {
        out.push_back(cont -> get_digital(button));
        out.push_back(cont -> get_digital_new_press(button));
    }
    prev = curr;
    curr = out;
    return(out);
}

std::vector<bool> keejLib::Controller::getReleased() {
    std::vector<bool> out;
    if (prev.size() != curr.size()) {
        return std::vector<bool>(30, 0);
    }
    for (int i = 0; i < curr.size(); i++) {
        if (prev[i] && !curr[i]) {
            // std::cout << "hi" << std::endl;
        }
        // std::cout << (prev[i] && !curr[i]) << std::endl;
        out.push_back(prev[i] && !curr[i]);
    }
    return out;
}
//https://www.desmos.com/calculator/puepnlubzh
double keejLib::Controller::curve(double x, double scale) {
    if (scale != 0) {
        return(pow(2.718, (scale * ((std::fabs(x) - 127))) / 1000 ) * x);
    }
    return x;
}

keejLib::ChassVelocities keejLib::Controller::drive(int direction, driveMode mode) {
    double lStick = curve(cont -> get_analog(ANALOG_LEFT_Y) * direction, leftCurve);
    double rStick;
    switch(mode) {
        case arcade:
            rStick = curve(cont ->get_analog(ANALOG_RIGHT_X), rightCurve);
            return {lStick + rStick, lStick - rStick};

        case tank:
            rStick = curve(cont -> get_analog(ANALOG_RIGHT_Y), rightCurve);
            return {lStick, rStick};
        
        case reverseArcade:
            rStick = curve(cont -> get_analog(ANALOG_RIGHT_Y) * direction, leftCurve);
            lStick = curve(cont ->get_analog(ANALOG_LEFT_X), rightCurve);
            return {rStick + lStick, rStick - lStick};
        case curvature:
            rStick = curve(cont ->get_analog(ANALOG_RIGHT_X), rightCurve);
            if (lStick == 0) {
                return {lStick + rStick, lStick - rStick};
            }
            double left = lStick + (std::abs(lStick) * rStick) / 127.0;
            double right = lStick - (std::abs(lStick) * rStick) / 127.0;
            return {left, right};
    }
    return {0,0};
}

void keejLib::Controller::setCurves(double left, double right) {
    leftCurve = left;
    rightCurve = right;
}