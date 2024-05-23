#pragma once
#include "units.h"

using namespace units::time;

namespace keejLib {
    class Stopwatch {
        private:
            millisecond_t start;
        public:
            Stopwatch();
        
            void reset();
            millisecond_t getTime();
    };
}