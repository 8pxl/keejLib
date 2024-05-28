#pragma once
namespace keejLib {
    class Stopwatch {
        private:
            int start;
        public:
            Stopwatch();
            void reset();
            int elapsed();
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