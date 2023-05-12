# keejLib !!
the best lib for pros fr

**features**
-
---
- motor wrapper + groups (support for diffy groups)
- piston wrapper + groups
- controller wrapper
- ezpz generic pid
- muy util (very useful!)
- chassis class
  - pid drive
  - pid turn
  - auto straight
  - arc turn
  - odom
  - 1d motion profiling (asym trapezoidal)
  - pid move to point
  - boomerang controller

**usage**
-
to use keejLib, simply clone this repository. everything is under the `lib` namespace.

**motor groups**

---

motor groups can be initilized using the port number of each motor.

to initlize the two types of motor groups:
```cpp
lib::mtrs intakeMtrs({1,2});
lib::diffy chassMtrs({3,4,5,6});
```
`mtrs` allows for the grouping together of any ammount of motors. it provides methods to spin, stop, etc. each motor in a group. 

`lib::diffy` is a differential motor group. it allows the spinning of each half of the motor group with distinctive velocities. these inherit all methods from normal motor groups. 

diffy motor groups must consist of an even amount of motors. when initializing *2n* motors, the first *n* motors are paired together, the second *n* the same.


**usage**

```cpp
intakeMtrs.spin(127);

chassMtrs.spin(90);
chassMtrs.spinDiffy(127,-100);
```

`spin` methods work for both types of groups, however the `spinDiffy` method is unique to differential groups. in this example, one half of the chassis would be given 127 volts, the other half -100.

**controller**

---
keejLib also extends the controller through an additional wrapper class. this adds concise acessing of the values of each button on the controller, built in selecting tool, and simple drive curves with support for arcade and tank.

lets take a look at some example driver code written using these features. (2496R 2023 worlds code)

```cpp
void keej()
{
    std::vector<bool> cont = robot::controller.getAll(ALLBUTTONS);
    chass.spinDiffy(robot::controller.drive(1, util::controller::arcade));
    bool cataIdle = cata::curr == cata::idle;

    if(cont[NL1]) cata::fire();
    if(cont[NR2]) tsukasa.toggle();
    if(cont[NR1] && tsukasa.getState()) tsukasa.toggle();
    if(cont[R1] && cataIdle) itsuki.spin(127);
    else if(cont[L2]) itsuki.spin(-127);
    else if(cataIdle) itsuki.stop('c');
    if(cont[NUP]) expansion.toggle();
}
```

firstly, a list of the state of each controller button is created using the `getAll` method.
