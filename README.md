CarND · T2 · P5 · Model Predictive Control (MPC) Project
========================================================

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

<img src="output/images/004 - Simulator Rotated.png" alt="Extended Kalman Filter visualization on the simulator." />


Project Overview
----------------

In this project...

TODO

To test it, [Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases) need to be used. The latest version of `main.cpp` used to run this project without the simulator can be found [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/blob/06cbc9967bc62592723eef99b8c8035e4a22ea7b/src/main.cpp).

If you are looking for Udacity's started code project, you can find it [here](https://github.com/udacity/CarND-MPC-Project).


Dependencies
------------

- [Udacity's Self Driving Car Simulator](https://github.com/udacity/self-driving-car-sim/releases)
- [`cmake >= 3.5`](https://cmake.org/install/)
- `make >= 4.1` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`3.81` (Windows)](http://gnuwin32.sourceforge.net/packages/make.htm)
- `gcc/g++ >= 5.4` (Linux / [Mac](https://developer.apple.com/xcode/features/)), [`MinGW` (Windows)](http://www.mingw.org/)
- [`uWebSockets` commit `e94b6e1`](https://github.com/uWebSockets/uWebSockets). See the following section for installation instructions and additional details.
- [`Eigen`](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
- [`Ipopt`](https://projects.coin-or.org/Ipopt) and [`CppAD`](https://www.coin-or.org/CppAD/). See the following section for installation instructions and additional details.


Installation
------------

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets):

- `install-mac.sh` for Mac.
- `install-ubuntu`for either Linux or [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) (**please, make sure it is updated**).

For Windows, Docker or VMware coulso also be used as explained in the [course lectures](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77). Details about enviroment setup can also be found there.

If you install from source, checkout to commit `e94b6e1`, as some function signatures have changed in `v0.14.x`:

    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1

See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

**Regarding `Ipopt` and `CppAD`**, please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.


Build & Run
-----------

Once the install is complete, the main program can be built and run by doing the following from the project top directory:

1. Create a build directory and navigate to it: `mkdir build && cd build`
2. Compile the project: `cmake .. && make`
3. Run it: `./MPC`

Or, all together (from inside the `build` directory): `clear && cmake .. && make && ./MPC`

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).


Reflection
----------

### The model

*Student describes their model in detail. This includes the state, actuators and update equations.*

The kinematic model uses the following state and actuators variables, directly coming from the simulator or calculated:

**STATE (INPUT):**

- `x`: Vehicle's X position in the map's coordinate system. Sent by the simulator as `x`.
- `y`: Vehicle's Y position in the map's coordinate system. Sent by the simulator as `y`.
- `v`: Vehicle's current speed. Sent by the simulator as `speed`.
- `cte`: Vehicle's cross-track error. Calculated using `ptsx` and `ptsy` (sent by the emulator), as explained next.
- `epsi`: Vehicle's orientation error Calculated using `ptsx` and `ptsy` (sent by the emulator), as explained next.

In order to calculate the CTE and EPSI values (`main.cpp:215-249`), the waypoints X and Y coordinates are transformed from the map's coordinate system to the vehicle's coordinate system and then a 3rd degree polynomial is fitted to them, resulting in:

    const double CTE = coeffs[0]; // f(px) = f(0) = coeffs[0]
    const double EPSI = -atan(coeffs[1]); // psi - f'(px) = 0 - f'(0) = coeffs[1]

It's worth mentioning that these values are also updated to account for the latency, as I will explain in the last point of this section.

**ACTUATORS (OUTPUT):**

- `d`: Vehicle's current steering angle.
- `a`: Vehicle's current acceleration/throttle.

In order to find the optimal value for `d` and `a`, and objective function that combines the squared sum of following terms will be minimized using Ipopt (`MPC.cpp:90-118`):

**REFERENCE STATE COST:**

Costs associated to the difference between the desired optimal state and the estimated one. The get really large weight in comparison to the other terms:

- CTE: Cross-track error.
- EPSI: Orientation error.
- Speed difference, between the estimated speed and the reference/target speed.

**CONTROL COST:**

To minimize the use of actuators. Set to small values as it is not a priority, in this case, to reduce the use of actuators.

- Delta: Steering angle.
- Delta change: To minimize the gap between sequential actuations to achieve temporal smoothness. Set to a relatively large value to follow a relatively smooth trajectory around the track.
- Acceleration/throttle.
- Acceleration/throttle change: Again, to minimize the gap between sequential actuations to achieve temporal smoothness, but set to a smaller value, as we favor trajectory smoothness versus speed smoothness.

The weights assigned to each term has been manually tuned by trial and error.

Also, the following equations are used to calculate the state at timestep `t` given the state at timestep `t - 1` (`MPC.cpp:165-189`):

    x1 = x0 + v0 * cos(psi0) * dt
    y1 = y0 + v0 * sin(psi0) * dt
    psi1 = psi0 + v0/Lf * delta0 * dt
    v1 = v0 + a0 * dt
    cte1 = f(x0) - y0 + v0 * sin(epsi0) * dt
    epsi1 = psi0 - f'(x0) - v0/Lf * delta0 * dt
    
Where `Lf = 2.67` is the distance from the front wheels to the vehicle's center of gravity (CoG), as provided by Udacity. Also, note a sign has been changed in the last one to make it work in the emulator.


### Timestep Length and Elapsed Duration (N & dt)

*Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.*

I'm using `N = 10` and `dt = 0.1s` or `dt = 100ms`, as these values have been suggested during the Q&A session and on the forums, so I just kept those as I assumed focusing on tuning the weights instead would have a greater influence on the final result.

Anyway, as the latency is `100ms`, it doesn't make too much sense to use a `dt` value smaller than that.


### Polynomial Fitting and MPC Preprocessing

*A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.*

Waypoints preprocessing already explained in the first point. State update to account for latency is explained in the next one.


### Model Predictive Control with Latency

*The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.*

In order to account for the latency, the state is updated as follows (`main.cpp:256-262`):

    x = v * lag
    y = 0 (as psi = 0 from the vehicle's coordinate system)
    psi = -v * lag * delta / Lf
    v = v + a * lag
    cte = cte + v * sin(epsi) * lag
    epsi = epsi - v * lag * delta / Lf

Also, a not too high target speed of 50 MPH has been set and an emergency breaking system has been implemented, which would actuate the breaks at their maximum (`throttle = -1`) if a combination of CTE and EPSI grows too much while going at speeds greater than 50 MPH.


## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
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

# Websocket Data

This document describes the JSON object send back from the simulator command server.

Fields:

* `ptsx` (Array<float>) - The global x positions of the waypoints.
* `ptsy` (Array<float>) - The global y positions of the waypoints. This corresponds to the z coordinate in Unity
since y is the up-down direction.
* `psi` (float) - The orientation of the vehicle in **radians** converted from the Unity format to the standard format expected in most mathemetical functions (more details below).
* `psi_unity` (float) - The orientation of the vehicle in **radians**. This is an orientation commonly used in [navigation](https://en.wikipedia.org/wiki/Polar_coordinate_system#Position_and_navigation).
* `x` (float) - The global x position of the vehicle.
* `y` (float) - The global y position of the vehicle.
* `steering_angle` (float) - The current steering angle in **radians**.
* `throttle` (float) - The current throttle value [-1, 1].
* `speed` (float) - The current velocity in **mph**.


### `psi` and `psi_unity` representations

`psi`

```
//            90
//
//  180                   0/360
//
//            270
```


`psi_unity`

```
//            0/360
//
//  270                   90
//
//            180
```
