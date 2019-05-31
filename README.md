# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Overview
The purpose of this project was to design a PID controller and tweak the PID hyperparameters by applying the general processing flow as described in the lessons.
The simulator provides cross-track error (CTE), speed, and steering angle data. The PID controller operates steering and throttle commands to drive the car reliably around the track.


## Dependencies
* cmake >= 3.5
* make >= 4.1(mac, linux), 3.81(Windows)
* gcc/g++ >= 5.4
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.


## Rubic Points
### Compilation
* Your code should compile.
No error message.

### Implementation
* The PID procedure follows what was taught in the lessons.
PID procedure is coded in src/PID.cpp and src/PID.h. 
PID coefficients(Kp, Ki, Kd) are moved from private section to public section in PID.h.
"PID::UpdateError" computes proportional, integral and derivative errors, and "PID::TotalError" returns the total error multiplying each error and its coefficient.

### Reflection
* Describe the effect each of the P, I, D components had in your implementation.
The test simulation moives of P-control, PD-control and PID-control are stored in "movie" folder.

- P(proportional)-control
consists of car's distance from the center of lane(=CTE) multiplied by its coefficient.If the car is far right it steers hard to the left, so the car is most likely to occilate around the center of the lane. (please refer to the movie:P-control.mov)

- PD(proportional, derivative)-control
consists of P-control and D-control. The advantage of D-control is the recovery of sudden error increase, such as sharp curve. In the PD-control movie(1. PD-Ctrl.mov), we can see big improvement compared to the movie of P-control(0. P-Ctrl.mov)

- PID(proportional, integral and derivative)-control
has I-control in addition to PD-control. The I-control reduces an unknown offset which prevents from reaching the center line. By comparing the position of the car in PD-control movie and PID-control (please run the simulator), we can clearly see the superiority of PID-control.

* Describe how the final hyperparameters were chosen.
(main.cpp, Line 111- 183)
Initial value of PID coefficients are derived from manual tweaking. 
For twiddling, I slightly modify the algorithm of twiddle taught in the lessons, by utilizing bisection method.
Firstly, coefficient of P-controller(Kp) is to be optimized alone, then proceeds to adjustment of coefficient of D-controller(Kd).
Lastly,  coefficient of I-controller(Ki) will be tweaked for completion.
The derived optimal parameters are as follows, [ Kp, Ki, Kd ] = [0.16, 0.0018, 1.35]

### Simulation
* The vehicle must successfully drive a lap around the track.

- Throttle Control
To control the throttle valve is almost equal to acceleration/deceleration control.
Below is basic concept of throttle control logic I implemented.
-- Acceleration is done only when the car is stable considering cars's deviation from lane center, car's agnle and steering value.
-- In the speed range between o and half of the limit, relatively strong acceleration is allowed.
-- From half up until limit, mild acceleration is being done.
-- When car is out of stability and quick deceleration is needed for recovery, strong deceleration will be done in propotion to steer_value.

Thanks to the optimized PID parameters and safe throttle control logic, the car can successfully drives around the track.
