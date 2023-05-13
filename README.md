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
-
---

motor groups can be initilized using the port number of each motor.

to initlize the two types of motor groups:
```cpp
lib::mtrs intakeMtrs({1,2});
lib::diffy chassMtrs({3,4,5,6});
```
`lib::mtrs` allows for the grouping together of any ammount of motors. it provides methods to spin, stop, etc. each motor in a group. 

`lib::diffy` is a differential motor group. it allows the spinning of each half of the motor group with distinctive velocities. an obvious usecase for this is a chassis, where you dont always want both sides to spin with the same velocity. diffy groups inherit all methods from normal motor groups. 

diffy motor groups must consist of an even amount of motors. when initializing *2n* motors, the first *n* motors are paired together, the second *n* the same.


**usage**

```cpp
intakeMtrs.spin(127); //normal mtr group
chassMtrs.spin(90); //diffy mtr group
chassMtrs.spinDiffy(127,-100);
```

`spin` methods work for both types of groups, however the `spinDiffy` method is unique to differential groups. in this example, one half of the chassis would be given 127 volts, the other half -100.

**controller**
-
---
keejLib also extends the controller through an additional wrapper class. this adds concise acessing of the values of each button on the controller, a built in selecting tool, and simple drive curves with support for arcade and tank.

to initilize a `lib::controller` object, simply pass in an existing controller to the constructor, like so
```cpp
pros::Controller controller(pros::E_CONTROLLER_MASTER);
lib::controller controller(glb::controller);
```
lets take a look at some example driver code written using these features. (2496R Worlds 2023)

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

firstly, a list of the state of each controller button is created using the `getAll` method, which takes a list of buttons to check and returns each value. this vector sorts the value of each button by indicies. (it is recommended to simply get the value of all buttons using the exisitng `ALLBUTTONS` macro, as this is most convenient to manage)

to make acessing each value easier, an enum was created to map button names to their corresponding index.
```
L1 = 0,
NL1 = 1,
L2 = 2,
NL2 = 3,
R1 = 4,
NR1 = 5,
R2 = 6,
NR2 = 7,
UP = 8,
NUP = 9,
DOWN = 10,
NDOWN = 11,
LEFT = 12,
NLEFT = 13,
RIGHT = 14,
NRIGHT = 15,
X = 16,
NX = 17,
B = 18,
NB = 19,
Y = 20,
NY = 21,
A = 22,
NA = 23 
```
note: N denotes a 'new' press of the following button.

```cpp
std::vector<bool> cont = robot::controller.getAll(ALLBUTTONS);

if(cont[A]) std::cout << "button a was pressed!" << std::endl;

if(cont[NL1]) std::cout << "new button L1 press detected!" << std::endl;
```
this demonstrates how the value of each button can be acessed utilizng the defined enumeration. (the previous enumeration is only valid when utilized on a vector that contains the value of all buttons in the specified order)

**controller - drive**

an additional `drive` method helps to simplify code to move the robot using the joysticks. 

the `drive` method will return a vector of length two, which will contain the output voltages to the left and right sides of the drivebase given the controller inputs. this method takes in the direction to drive (1,-1) and the type of drive method (`util::controller::arcade`, `util::controller::tank`)

the vector return value pairs perfectly with the `spinDiffy` method in diffy groups (which was previously talked about) all in all, drive code for the robot can be reduced to just one line:

```cpp
    chass.spinDiffy(robot::controller.drive(1, util::controller::arcade)
```

aditionally, keejLib allows for the option to 'curve' the joystick inputs using the following equations: `https://www.desmos.com/calculator/puepnlubzh`

```cpp
robot::controller.setCurves(0, 8);
```
this will set the t value to be used for the left and right joysticks. (higher value results in a greater degree of input scaling)