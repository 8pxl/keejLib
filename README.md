# keejLib
a quality of life util lib for pros! 
 
features motor groups, and various util functions.

<font size = "1">(thank you evana thomson for the inspiration!)</font>
# usage
keejlib is intended to be added to an existing pros project, simply drag the lib folder into your project and add `#include lib/lib.hpp` to include the library. everything is under the `lib` namespace.

---

<font size = "4">**initialize motor groups**
</font>
```cpp
pros::Motor mtr1(1, pros::E_MOTOR_GEARSET_06, true); 
pros::Motor mtr2(2, pros::E_MOTOR_GEARSET_06, true);

//normal motor group
lib::mtrs motorGroup(std::vector<pros::Motor>{mtr1, mtr2});

//differential motor group (motors need to spin diff velocities)
lib::diffy diffyGroup(std::vector<pros::Motor>{mtr1, mtr2});
```
differential motor groups must consist of an even amount of motors. when initializing *2n* motors, it will consider the first *n* motors as paired together, and the second *n* as well. each "pair" of motors can be spun at a different voltage.


<font size = "4">**motor group usage**
</font>
```cpp
motorGroup.spin(127);

diffyGroup.spin(90);
diffyGroup.spinDiffy(127,-100);
```
<font size = "2">**motor group methods are as follows:**</font>

* `spin(double volts)`
* `stop(char brakeMode)` ***('c', 'b', 'h')***
*  `setBrake(char brakeMode)`
* `reset()`
* `getSpeed()`
* `getRotation()`

<font size = "2">**diffy only:**
</font>
* `spinDiffy(double rvolt, double lvolt)` 
* `getDiffy()`

diffy groups inherit all methods from normal motor groups. 

---

<font size = "4">**imu**
</font>

keejlib also has its own imu class in order to get radian heading, and to set an initial heading. (ex. if the robot starts facing left, you can set the initial heading to 90 degrees.)

<font size = "4">**usage**
</font>

```cpp
pros::Imu imu(1);

lib::imu(imu, 0); //second parameter is the initial rotation.

double currHeading = lib::imu.radHeading();
```
---


<font size = "4">**opcontrol**
</font>

code is pretty self explanatory, example code follows:
![listener](https://user-images.githubusercontent.com/56054380/200656086-fbf8a1fd-9f79-428c-a6e3-d0f86537021d.png)


<font size = "4">**chassis**
</font>
```cpp
lib::chassis chass(diffyGroup, imu);

chass.updatePos(2,3);
```
this is only meant as a template class, add methods as you please. an example drive function is included.

---
<font size = "4">**util**
</font>

keejlib also features various utility functions and classes, most are trivial to use. example usage is as follows (copy and pasted from the default drive function)

```cpp
void lib::chassis::drive(double target, double timeout, util::pidConstants constants)
{
  double error = target;
  util::timer timer;
  util::pid pidController(constants, target);
  chass.reset();

  while(timer.time() < timeout)
  {
    error = target - chass.getRotation();
    chass.spin(pidController.out(error));
  }

  chass.stop('b');
}
```



