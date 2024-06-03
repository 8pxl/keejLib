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
        
        double dist(pt a);
    };
    
    struct Pose {
        pt pos;
        double heading;
    };
    
    template <typename T>
    int sign(T x);
    
    int dirToSpin(double target, double current);
    double angError(double target, double current);
    double toRad(double deg);
    double toDeg(double rad);
    double toStandard(double deg);
    double fromStandard(double rad);
}