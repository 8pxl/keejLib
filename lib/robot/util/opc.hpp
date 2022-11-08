#include "main.h"
#include "util.hpp"
#include <vector>

namespace lib
{
    class listener
    {
        private:
            std::vector<util::action> actions;
            pros::Controller con;

        public:
            listener(pros::Controller controller, std::vector<util::action> action) : con(controller), actions(action){}

            void init(pros::Controller controller, std::vector<util::action> action)
            {
                con = controller;
                actions = action;
            }
            void listen()
            {
                for (int i = 0; i < actions.size(); i++)
                {
                    if(con.get_digital(actions[i].button))
                    {
                        actions[i].pressed();
                    }

                    else
                    {
                        actions[i].unpressed();
                    }
                }
            }
    };
}   