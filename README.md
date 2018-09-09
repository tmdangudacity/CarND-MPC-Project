# Model Predictive Control Project
Self-Driving Car Engineer Nanodegree Program

---
## Overview
This repository contains all the code implemented for the Model Predictive Control Project in Udacity's Self-Driving Car Nanodegree.

## Submission
A completed version of a Model Predictive Controller [MPC.cpp](./src/MPC.cpp) was implemented in C++ and is located in the [src](./src/) directory. The controller operates with a set of waypoints, vehicle's position, heading and speed in a global coordinate frame at each time step.


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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Test results
The MPC controller was tested with the simulation of the Lake track.

The vehicle successfully completed a full lap around the Lake track with a maximum speed reaching 92 mph.

The debug output was captured in [Test result](./results/MPC_Run.txt). 

The footage of the test run was recorded in [MPC_Run.mp4](./results/MPC_Run.mp4).

## [Rubric](https://review.udacity.com/#!/rubrics/896/view) points
### Compilation
The code is compiled without errors or warnings.

```
    make clean; make
    [ 33%] Building CXX object CMakeFiles/mpc.dir/src/MPC.cpp.o
    [ 66%] Building CXX object CMakeFiles/mpc.dir/src/main.cpp.o
    [100%] Linking CXX executable mpc
    [100%] Built target mpc

```

### Implementation

** The Model: **

The vehicle's kinematics model (including cross track error and heading error terms) which was used in the implementation is the same as the one provided in the lessons. The update equations are as following:

```

    x[t+1]    = x[t]   + v[t] * cos(psi[t]) * dt
    y[t+1]    = y[t]   + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] * delta[t] * dt / Lf
    v[t+1]    = v[t]   + a[t] * dt
    cte[t+1]  = cte[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = epsi[t] + v[t] * delta[t] * dt / Lf
```
where

t: timestamp

x and y: vehicle coordinates at a given time stamp

psi: vehicle heading

v: vehicle ground speed

delta: vehicle steering angle actuation

a: vehicle acceleration actuation

Lf: distance from the front steering axle to the vehicle's centre of gravity

cte: vehicle's cross track error

epsi: vehicle's heading error

** Number of timestep N and elapsed duration dt: **

The values of timesteps N and elapsed duration dt are to minimise the cost function for each iteration of vehicle and path inputs.

The cost function in this implementation includes cross-track error, heading error and speed error relative to a pre-set reference speed. It also includes minimal use of actuators and minimal change between subsequent actuations.

The code for the cost function is in [MPC.cpp](./src/MPC.cpp) as follows:
```
            // The part of the cost based on the reference state.
            for(unsigned int t = 0; t < N; ++t)
            {
                fg[0] += CppAD::pow(vars[cte_start  + t], 2);
                fg[0] += CppAD::pow(vars[epsi_start + t], 2);
                fg[0] += CppAD::pow((vars[v_start   + t] - reference_speed), 2);
            }

            // Minimize the use of actuators.
            for(unsigned int t = 0; t < (N - 1); ++t)
            {
                fg[0] += CppAD::pow(vars[delta_start + t], 2);
                fg[0] += CppAD::pow(vars[a_start     + t], 2);
            }

            // Minimize the value gap between sequential actuations.
            for(unsigned int t = 0; t < (N - 2); ++t)
            {
                fg[0] += (reference_speed * reference_speed) * CppAD::pow((vars[delta_start + t + 1] - vars[delta_start + t]), 2);
                fg[0] += CppAD::pow((vars[a_start + t + 1] - vars[a_start + t]), 2);
            }

```

To smooth out the steering actuation, the cost of the steering angle change is scaled-up by the  square of the reference speed. The higher the reference speed the smaller the change of angle of steering should be to avoid vehicle overshooting reference path.

Different combinations of values of N and dt were tested.

Testing showed that a larger value of N (for example N = 25) caused too large of change of steering angle actuation when reference path under the vehicle changed to a higher curvature at high speed. A larger number of N also took longer for the optimisation to run and the solution could be late before the next update of vehicle states at high speed.

A small value of N would cause the solution not be optimal and the cross track error and heading error did not converge well at the end of an iteration.

The best combination of N and dt was N = 10 and dt = 0.05 second.


** Polynomial Fitting and MPC Preprocessing: **

The reference waypoints were first transformed from the global coordinate frame to the local frame at the  vehicle position with the X-axis along the vehicle heading and the Y-axis to the left.

A third-order polynomial was then fitted to the transformed reference waypoints. Third order polynomial was selected to best fit trajectories for most road conditions.

The vehicle crosstrack and heading errors were computed with the vehicle position at the origin of the local frame and vehicle heading of zero.

The code of the transformation is in [main.cpp](./src/main.cpp) as follows:

```

          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double v   = j[1]["speed"];

          vector<double> local_ptsx(ptsx.size());
          vector<double> local_ptsy(ptsy.size());

          Eigen::VectorXd reference_x = Eigen::VectorXd(ptsx.size());
          Eigen::VectorXd reference_y = Eigen::VectorXd(ptsy.size());

          double temp_x = 0.0;
          double temp_y = 0.0;

          for(unsigned int i = 0; i < ptsx.size(); ++i)
          {
              temp_x = ptsx[i] - px;
              temp_y = ptsy[i] - py;

              local_ptsx[i]  =  temp_x * cos(psi) + temp_y * sin(psi);
              reference_x[i] =  local_ptsx[i];

              local_ptsy[i]  = -temp_x * sin(psi) + temp_y * cos(psi);
              reference_y[i] = local_ptsy[i];
          }

          Eigen::VectorXd coeffs = polyfit(reference_x, reference_y, 3);

          //In ENU counter clockwise positive
          double cte  = 0.0 - coeffs[0];
          double epsi = 0.0 - atan(coeffs[1]);
```

** Model Predictive Control with Latency: **

To handle the actuation latency, a projection forward of vehicle states was implemented in [MPC.cpp](./src/MPC.cpp) as follows:

```

    Eigen::VectorXd MPC::ProjectState(const Eigen::VectorXd& state, const Eigen::VectorXd& coeffs, double dt) const
    {
        Eigen::VectorXd projected(state.size());

        if(dt > 0.0)
        {
            double x    = state[0];
            double y    = state[1];
            double psi  = state[2];
            double v    = state[3];
            double cte  = state[4];
            double epsi = state[5];

            double delta_psi = v * m_delta * dt / Lf;

            double x2    = x    + v * cos(psi) * dt;
            double y2    = y    + v * sin(psi) * dt;
            double psi2  = psi  + delta_psi;
            double v2    = v    + m_a * dt;
            double cte2  = cte  + v * sin(epsi) * dt;
            double epsi2 = epsi + delta_psi;

            projected << x2, y2, psi2, v2, cte2, epsi2;

            std::cout << "ProjectState with delay " << dt << std::endl;
        }
        else
        {  
            projected = state;

            std::cout << "ERROR! ProjectState with wrong delay " << dt << std::endl;
        }

        return projected;
    }

```
The code above projects the vehicle's states at the beginning of each iteration ahead using the cached value of steering angle (m_delta) and acceleration (m_a) from the last iteration.  The value of m_delta and m_a were initialized to zero at the beginning.

### Simulation result

In order for the vehicle to steer at the highest possible speed around the track and also handle different value of path curvature, curvature adaptation for vehicle speed was implemented.

The reference speed for each model prediction iteration is scaled from a maximum speed using the value of path curvature near the vehicle position.

Curvature of a graph function y = f(x) is calculated as:

k = y'' / (1 + (y')**2) ** (3/2)

Path curvature of a thir-order polynomial is implemented in [MPC.cpp](./src/MPC.cpp) as follows:

```

    //First derivative of cubic polynomial
    AD<double> Poly3Dot1Eval(const Eigen::VectorXd& coeffs, AD<double> x)
    {
        return (coeffs[1] + 2.0 * x * coeffs[2] + 3.0 * x * x * coeffs[3]);
    }

    //Second derivative of cubic polynomial
    AD<double> Poly3Dot2Eval(const Eigen::VectorXd& coeffs, AD<double> x)
    {
        return (2.0 * coeffs[2] + 6.0 * x * coeffs[3]);
    }

    //Curvature calculation
    AD<double> Curvature(const Eigen::VectorXd& coeffs, AD<double> x)
    {
        AD<double> d2 = Poly3Dot2Eval(coeffs, x);
        AD<double> d1 = Poly3Dot1Eval(coeffs, x);

        return ( d2 / pow( (1.0 + d1 * d1), 1.5) );
    }
```

Curvature adaptation for vehicle speed was implemented in [MPC.cpp](./src/MPC.cpp) as follows:

```

    //Curvature adaptation for reference speed
    AD<double> start_curvature = Curvature(coeffs, vars[x_start]);
    AD<double> reference_speed = MAX_SPEED / (1.0 + CURVATURE_SCALE * fabs(start_curvature));
    
```
where MAX_SPEED is set to 100mph and CURVATURE_SCALE is a tuneable scale factor of curvature and set to 25.

For example, if the value of the curvature scale factor is 25 and the path curvature radius is 25 metres, the reference speed would be scaled to half of the maximum speed, becoming 50 mph.

The vehicle successfully drove around the simulation of the Lake track with no tire leaving the drivable portion of the track surface. The car did not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe.

The footage of a completed lap is in [MPC_Run.mp4](./results/MPC_Run.mp4).









