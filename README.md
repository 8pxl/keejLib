# keejLib !!
the best lib for pros fr


<font size = 6>**features** </font>
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


<font size = 6>**hardware abstractions**</font>
-
---


**motor groups**
-


motor groups can be initialized using the port number of each motor.


to initialize the two types of motor groups:
```cpp
lib::mtrs intakeMtrs({1,2});
lib::diffy chassMtrs({3,4,5,6});
```
`lib::mtrs` allows for the grouping together of any amount of motors. it provides methods to spin, stop, etc. each motor in a group.


`lib::diffy` is a differential motor group. it allows the spinning of each half of the motor group with distinctive velocities. an obvious use case for this is a chassis, where you dont always want both sides to spin with the same velocity. diffy groups inherit all methods from normal motor groups.


diffy motor groups must consist of an even amount of motors. when initializing *2n* motors, the first *n* motors are paired together, the second *n* the same.




**usage**


```cpp
intakeMtrs.spin(127); //normal mtr group
chassMtrs.spin(90); //diffy mtr group
chassMtrs.spinDiffy(127,-100);
```


`spin` methods work for both types of groups, however the `spinDiffy` method is unique to differential groups. in this example, one half of the chassis would be given 127 volts, the other half -100.


**piston groups**
-
similarly, a separate piston group was also created. although the need to pair multiple pistsons together is not very needed, the toggle method of the piston class is very useful.


piston groups are not initialized with the port number, but instead by passing in an existing `pros::ADIDigitalOut` object.


```cpp
 pros::ADIDigitalOut piston('B');
 lib::pis tsukasa({piston}, false);
```


the piston constructor also takes in an additional boolean to indicate the starting value of the piston. the state of the piston will be set to this starting value.


`toggle`, `getState, and setState` methods do as their names suggest.


**controller**
-
---
keejLib also extends the controller through an additional wrapper class. this adds concise accessing of the values of each button on the controller, a built in selecting tool, and simple drive curves with support for arcade and tank.


to initialize a `lib::controller` object, simply pass in an existing controller to the constructor, like so
```cpp
pros::Controller controller_pros(pros::E_CONTROLLER_MASTER);
lib::controller controller_lib(controller_pros);
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


firstly, a list of the state of each controller button is created using the `getAll` method, which takes a list of buttons to check and returns each value. this vector sorts the value of each button by indices. (it is recommended to simply get the value of all buttons using the existing `ALLBUTTONS` macro, as this is most convenient to manage)


to make accessing each value easier, an enum was created to map button names to their corresponding index.
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
this demonstrates how the value of each button can be assessed utilizing the defined enumeration. (the previous enumeration is only valid when utilized on a vector that contains the value of all buttons in the specified order)


**controller - drive**


an additional `drive` method helps to simplify code to move the robot using the joysticks.


the `drive` method will return a vector of length two, which will contain the output voltages to the left and right sides of the drivebase given the controller inputs. this method takes in the direction to drive (1,-1) and the type of drive method (`util::controller::arcade`, `util::controller::tank`)


the vector return value pairs perfectly with the `spinDiffy` method in diffy groups (which was previously talked about) all in all, drive code for the robot can be reduced to just one line:


```cpp
   chass.spinDiffy(robot::controller.drive(1, util::controller::arcade)
```


additionally, keejLib allows for the option to 'curve' the joystick inputs using the following equations: `https://www.desmos.com/calculator/puepnlubzh`


```cpp
robot::controller.setCurves(0, 8);
```
this will set the t value to be used for the left and right joysticks. (higher value results in a greater degree of input scaling)


**controller - select**

note - this method is blocking.

one issue that is often encountered is to have a quick and concise way to get user input. although this can be done using the brain screen, i've found it most efficient to use the controller.


the `select` method aims to add an easy way to select between any number of options. it does this by displaying each option one at a time on the controller screen, the options can be cycled using the arrow buttons on the controller, pressing the `a` button selects the current options. 

the `select` method takes for input a list of names to display, and returns the index of the name selected.

for example:

```cpp
int color = robot::controller.select({"blue", "red", "green"});`
```
this would initially display "blue" on the controller screen, upon pressing the right arrow key, the value displayed would change to "red", as expected. upon presssing "a" on the controller, the current option (red) will be selected, and the function will return 1. if instead "green" was selcted, 2 would be returned.



<font size = 6>**utility**</font>
-
---
an overview of each utility feature (view util.h for detailed arguments):


- `struct pidConstants`


 holds constants for a pid controller.
- `struct point`




 holds two doubles.


- `typedef point vec`


 alias for point struct, for code clarity.


- `struct robotConstants`
  
 holds constants for robot, used when initializing a `chassis` object


- `struct atns`

 holds a list of auton function pointers and a list of names.


- `class cubicBezier`

 class to represent a cubicBezier given 4 points


- `double dtr()`

 converts degrees to radians

- `double rtd()`
  
 converts radians to degrees

- `double sign()`

 returns 1 if positive, -1 otherwise.

- `double hypot()`

hypotenuse given 2 side lengths. has support for passing in 2 numbers or a `lib::point`


- `double absoluteAngleToPoint()`

 returns the angle between two points using inverse tangent

- `double dist()`

  returns the distance between two `point`s


now for the non trivial stuff...


**pid**
-


the `pid` class aims to create a reusable implementation of the pid controller (porportional integral derivative). it requires initializing using `pidConstants` and an initial error (which can generally be set to zero) once initilized. use the `out` method and pass in the current error to get the output of the PID control loop. 

example usage:
```cpp
  lib::pid pidController(constants, target);
  while true
  {
    double error = //get error somehow;
    double output = pidController.out(error);
  }
```
when passing error into the pid controller, it internally keeps track of the values needed to compute derivatve and integral.

**bezier**
-
keejLib also provides support to create cubic bezier curves. these curves are defined using 4 points. the bezier class has to evaluate the curve and its derivative at a given t value. 

the implemented method to approximate the length of the curve consists of subdividng the curve into an inputted ammount of straight line segments, and taking the summation. however this method is computationally slow, it is reccomended that another method be implemented. (on the todo list!)

**turn helpers**
-

the `dirToSpin` function takes the robots current heading and the robots desired heading as input, and ouputs 1 or -1 if the robot should spin clockwise or counterclockwise to reach the desired heading quickest.

the `minError` function takes the same inputs as the previous function, however it returns the minimum angluar error between the two.

**timer**
-

the timer function alows the user to create multiple timers to keep track of how much time has elapsed. think of it more like a stopwatch function.

to use:
```cpp
lib::timer t1;
int timeElapsed;
//do stuff
timeElapsed = t1.time();
t1.reset();
```
the `reset` method sets the time to 0.

<font size = 6>**chassis**</font>
-
---

there is a seperate class that handles all the movement for the chassis of the robot. to initialize:
```cpp
pros::Imu imu(5);
lib::diffy chassMtrs({1,2,3,4});

lib::chassis chass
(
    chassMtrs, 
    imu, 
    {6, 8}, 
    {
        .horizTrack = 0, 
        .vertTrack = 0,
        .trackDia = 0,
        .maxSpeed = 0,
        .fwdAccel = 0,
        .fwdDecel =  0,
        .revAccel =  0,
        .revDecel = 0,
        .velToVolt = 0
    }
);
```
the constructor takes for input:
- an existing `lib::diffy` object containing the chassis motors 
- an exisiting `pros::Imu` object
- a list of encoder ports (currently only supports 2)
- a `lib::robotConstants` instance containing all the robbot constants

**robot constants**

the robot constants needed are as follows. distance units do not matter as long as they stay consistent

for odom:
- horizTrack (distance to center of horizontal tracking wheel)
- vertTrack 
- trackDia (tracking wheel radius)

for motion profiling:
- maxSpeed (distance / 10ms) 
- fwdAccel (distance / 10ms^2)
- fwdDecel (different acceleration and decceleration rates may want to be used)
- revAccel (different constants depending on the direction the robot is traveling)
- revDecel
- velToVolt (conversion factor. distance / 10ms -> voltage applied to motor)