# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Reflection

Proportional-Integral-Derivative (PID) controller had to be implemented. PID controller
is a control feedback loop that continuously calculates the error between the desired and
measured value. 

In PID controller:

* P accounts for present values of the error. 
* I accounts for past values of the error compensates for systematic bias, e.g. steering wheel misalignment.
* D accounts for possible future values of the error e.g. it prevents overshoot.

Demo videos show the impact of different PID components:

1. [Vehicle oscilates when only P is used.](https://github.com/miharothl/CarND-PID-Control-Project/blob/master/demo/01-P.mov)
2. [Overshoot is prevented when both P and D are used. Steering is discerete.](https://github.com/miharothl/CarND-PID-Control-Project/blob/master/demo/02-PD.mov)
3. [Continous delayed steering when only I is used.](https://github.com/miharothl/CarND-PID-Control-Project/blob/master/demo/03-I.mov)
4. [Complete PID controler.](https://github.com/miharothl/CarND-PID-Control-Project/blob/master/demo/04-PID.mov)

Two controllers were used in order to drive the car around the track in the simulator. One to control
steering and one to control throttle.
 
Hyper-parameters Kp, Ki and Kd determine how much each of the component contributes to
the calculated error. They were set manually:

* steering PID - Kp = 0.06; Ki = 0.05; Kd = 2.0;
* throttle PID - Kp = 1.40; Ki = 10.0; Kd = 5.0;

The process of choosing steering hyper parameters:

1. Set speed to 40Mph. Set Kp so vehicle doesn't oscillates too much. Vehicle completes half of the track.
2. Speed 40Mph. Set Kd. Oscillation dampened. Steering is discrete. Late in sharp turns.
3. Speed 40Mph. Set Ki. Continuous steering, completes the track.
4. Speed 40Mph. Increased Ki to improve sharp turns. Increased speed to 60Mph. Completes track. Gentle turns.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
