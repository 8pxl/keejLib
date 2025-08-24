#pragma once
#include "main.h"
namespace keejLib {
    class Pis
    {
        private:
            std::vector<pros::adi::DigitalOut> pistons;
            bool state;
        
        public:
            Pis(std::vector<pros::adi::DigitalOut> p, bool s);
    
            void toggle();
            void setState(bool iState);
            bool getState();
    };
}