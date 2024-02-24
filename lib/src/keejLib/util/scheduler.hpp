#pragma once
#include "main.h"
#include "../include/keejLib/lib.h"
namespace lib {
    void lib::scheduler::run() {
        uint32_t time = pros::millis();
        auto it = upper_bound(tasks.begin(), tasks.end(), time, [](const uint32_t a, const task &b) {
            return (b.start < a);
        });

        if (it != tasks.end()) {
            for(auto q = tasks.begin(); q != it; q++) {
                q -> task();
                // pros::Task{[=]{q->task();}};
                tasks.erase(q);
            }
        }
    }

    void lib::scheduler::add(fptr item, uint32_t start) {
        tasks.insert({item, start + pros::millis()});
    }

    void lib::scheduler::init() {
        mainTask = new pros::Task {[=] {
            while (true) {
                run();
                pros::delay(10);
            }
        }};
    }
}
