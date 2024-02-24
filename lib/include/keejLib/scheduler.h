#pragma once
#include "util.h"

namespace lib {    
    class scheduler {
        private: 
            struct task {
                fptr task;
                uint32_t start;
            };

            struct cmp {
                bool operator()(const task &a, const task &b) const{
                    return a.start < b.start;
                };
            };
            pros::Task* mainTask = nullptr;
            std::multiset<task, cmp> tasks; 
        public:
            scheduler(){};
            void run();
            void add(fptr task, uint32_t start);
            void init();
    };
}
