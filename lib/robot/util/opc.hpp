#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "util.hpp"
#include <vector>

namespace lib
{
    class digital
    {
        public:
            virtual bool getState();
    };

    class controllerButton : public digital
    {
        private:
            pros::Controller con;
            pros::controller_digital_e_t button;

        public:
            controllerButton(pros::Controller cont, pros::controller_digital_e_t key) : con(cont), button(key){}

            bool getState()
            {
                return(con.get_digital(button));
            }
    };

    class limit : public digital
    {
        private:
            pros::ADIDigitalIn sensor;

        public:
            limit(pros::ADIDigitalIn limitSwitch) : sensor(limitSwitch) {}

            bool getState()
            {
                return(sensor.get_value());
            }
    };

    struct action
    {
        lib::digital* binaryIn;
        std::function<void(void)> pressed;
        std::function<void(void)> unpressed;

        action(lib::digital* digitalIn, std::function<void(void)> action, std::function<void(void)> defaultAction) : binaryIn(digitalIn), pressed(action), unpressed(defaultAction){}
    };

    class listener
    {
        private:
            std::vector<lib::action> actions;

        public:
            listener(int a){}
            listener(std::vector<lib::action> action) : actions(action){}

            void init(std::vector<lib::action> action)
            {
                actions = action;
            }

            void listen()
            {
                for (int i = 0; i < actions.size(); i++)
                {
                    if(actions[i].binaryIn->getState())
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