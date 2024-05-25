#pragma once
#include "units.h"

using namespace units::time;
using namespace units::length;

namespace keejLib {
    class Stopwatch {
        private:
            millisecond_t start;
        public:
            Stopwatch();
            void reset();
            millisecond_t getTime();
    };
    
    class EMA {
       private:
        double ka, last;
       public:
        EMA(double ka);
        double out(double val);
        double curr();
    };
    
    struct pt {
        inch_t x;
        inch_t y;
    };
}