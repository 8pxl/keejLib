#pragma once
namespace keejLib {
    class Stopwatch {
        private:
            double start;
        public:
            Stopwatch();
            void reset();
            double getTime();
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
        double x;
        double y;
    };
}