#pragma once
#include "main.h"
namespace keejLib {
    class Pis
    {
        private:
            std::vector<pros::ADIDigitalOut> pistons;
            bool state;
        
        public:
            Pis(std::vector<pros::ADIDigitalOut> p, bool s);
    
            void toggle();
            void setState(bool iState);
            bool getState();
    };
}