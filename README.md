# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Concept

A proportional–integral–derivative controller (PID controlleer) is a control loop feedback mechanism widely used in industrial control systems

 A PID controller continuously calculates an error value e(t) as the difference between a desired setpoint (SP) and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms. Term P is proportional to the current value of the SP − PV error e(t). Term I accounts for past values of the SP − PV error and integrates them over time to produce the I term. Term D is a best estimate of the future trend of the SP − PV error, based on its current rate of change.

## Coefficients

Kp: Is the Proportional Correction coefficient. It is responsible for the Direction and magnitude of the correction. However due to the momentum principle in control problems, Just using Kp will result in overshoot of the target state.
Ki: Is the Integral Correction Coefficient. It is responsible for the elimination of the residual steady-state error that occurs with a pure proportional controller while increasing the speed of achieving target state. The momentum principle can plague the I controller as well so it is also prone to overshoot and oscillation
Kd: Is the Differential Correction Coefficient. It is reponsbile for Dampening of the oscillation by being sensitive to the rate of change of instantaneous error rather than just pure magnitude or direction. Simply put, Kd will delay the achievement of target state but will also dampen and suppress the overshoot and oscillations.

## PID applied to the Self Driving Car

This project required control of a Self Driving Car in the Simulator using the PID loop on the steering. Another sentiment expressed in the project writeups was to see how fast could the car go in the simulator while not going off track. Towards this I also added a PID controller on the Target speed. Details of this are explained in the Subsequent Section. 

## Tuning Methodology

The Following steps were followed during the tuning Iterations

A Steering gain was tuned heuristically to get the car to drive safely for one lap using the Kp, Ki, Kd values in there sensible order of magnitudes namely 0.1, 0.01, 1.0
After the absolute values of the Kp, Ki, Kd were calculated that could complete one lap at 10 mph, the speed was gradually increased till the car could not complete one lap.

## Effect of Coefficients

If Kp is too high, Car will oscillate too much both in the steering as well as the speed domain. There will be constant overshoot which can result in vehicle going offtrack at points where the margin for error is low. If Kp is too low the Car will have a tendency to understeer at turns and will go off track in certain cases.

If Kd is too high, There is a lot of jitter in the system about the target position, If it is too low , The overshoot problem occuring from Kp, and Ki will not be dampened enough.

If Ki is too high, it can lead to historical error accumulation overpowering the instantaneous error and the rate of change components (Kp,Kd). If it is too low or absent, the controller will not be able to smoothen the correction curve.

## Final Parameter Choice

Since 2 PIDs were used which sort of impact each other, the tuning had to progress both simultaneously as well as in alternate cycles once driving at higher speeds was attained.

The Final Parameter set was as follows

Steering Kp, Ki, Kd : (0.127221000, 0.000018, 2.5)
Velocity Kp, Ki, Kd : (0.109170, 0.000754, 0.841226)

