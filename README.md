[image1]: PID.png



# CarND PID Control
Self-Driving Car Engineer Nanodegree Program

![Autonomous mode running with PID controller][image1] 

---

## Introduction
In this project, the PID controller is implemented for Autonomous driving mode. The purpose of PID control is for lane tracking and the controller is implemented using c++. The PID control signal is indeed steering angle in this case. After several PID tunings, the optimal PID gains are Kp = 0.2, Ki = 0.0004, and Kd = 2.5. With these PID gains, the vehicle is able to track the lane successfully without going out from the desired path. Note that, the error signal used in PID controller is "cte": cross-track error. 


## Effect from each PID Controller Gain
The PID control consists of proportional control gain (P), integral control gain (I) and derivative control gain (D). In this project, the (Kp, Kd, Ki) represents a set of P,I,D control gains respectively. 

- P gain helps improve faster tracking response (rise time) meaning the vehicle will try to steer to the lane center faster. P gain is tuned at first. Samll P gain results in slow rise time, meaning the vehilce will move towards to the lane center slowly. On the other hand, large P gain will result in faster steering to lane center, often P gain will introduce overshoot meaning the vehicle will move forward similar to sinusoidal path. 

- In order to decrese the overshoot, D gain is introduced secondly. Smaller overshoot means less oscilation (more smooth) around the lane center as vehicle moves forward. Large D gain may cause unstable response as the cte will grow to infinity, resulting in the vehicle will eventually stay out of the track. In this project, I found that using PD control will result in good tracking performance and good stability. However, the steady state error still remains in the case of PD control. 

- I control will help decrease steady state error. The cte error will be zero eventually for set-point tracking. The vehicle will try to stay at the lane center for path following as PID controller will try to drive the cte to zero. Using large I gain may also cause unstable response as well as P and D gain. 

When implement PID control, the control saturation limited is also introduced as well. The tuned optimal PID is stable so that it keeps the vehicle within the track. Failure of PID tuning may cause unstable response which the vehicle cte will grow infinite, meaning the vehicle will finally stay outside the track and unable the keep tracking the desired path.

## Tuning PID Controller
As mentioned earlier, I tuned the controller manually (trials and errors) starting from Kp, Kd, and Ki respectively. Alternatively, there are other methods for PID tuning as well such as using twiddle (introduced in class lesson), PID gain scheduling, etc. 




## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.



