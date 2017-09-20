# **Extended Kalman Filter**

**Udacity Self Driving Car Nanodegree - Project #6**

2017/9/19

## Overview

This project implements a **sensor fusion** algorithm to use **standard (linear)** and **extended (nonlinear)** [Kalman filter](https://en.wikipedia.org/wiki/Kalman_filter) (or LQE linear quadratic estimator) in C++ to estimate the state (position and velocity) of a moving object of interest with noisy LIDAR and RADAR measurements.

The motion model is a simple **linear Constant Velocity** model so the Kalman filter **Predict** step uses the standard (linear) Kalman filter equations.

For LIDAR measurements, the position is measured directly so linear Kalman filter equations can also be used for the **Update** step.  For RADAR measurements, the measurement is in polar coordinates so the conversion to state variable position and velocity is nonlinear and uses Extended Kalman filter equations by linearizing around the current state with a **Jacobian** matrix.

For each measurement received from the [Udacity Term 2 Simulator](https://github.com/udacity/self-driving-car-sim/releases), the program predicts the state for the current timestep, then updates the state with the RADAR or LIDAR measurement and calculated Kalman gain.  Sensor measurement noise covariance (R) and process noise covariance (Q) values are provided by Udacity for this project.

The implementation uses the following matrices and equations:

* x = estimated state
* P = state covariance matrix
* F = state transition matrix
* Q = process noise covariance matrix
* H = measurement matrix
* R = measurement noise covariance matrix
* z = measurement vector
* y = error between measurement and estimate from state
* S = pre-fit covariance matrix
* K = Kalman gain

**Standard Linear Kalman Filter Predict step equations**

* x = F * x
* P = F * P * Ft + Q

**Standard Linear Kalman Filter Update step equations**

* z_pred = H * x
* y = z – z_pred
* S = H * P * Ht + R
* K = P * Ht * Sinv
* x = x + (K * y)
* P = (I – K * H) * P

**Extended Nonlinear Kalman Filter Predict step equations**

*Note: These were not needed because motion model is linear*

* Fj = Jacobian of f(x) linearized at state x
* x = f(x)
* P = Fj * P * Fjt + Q

**Extended  Nonlinear Kalman Filter Update step equations**

* Hj = Jacobian of h(x) linearized at state x
* z_pred = h(x)
* y = z – z_pred
* S = Hj * P * Hjt + R
* K = P * Hjt * Sinv
* x = x + (K * y)
* P = (I – K * Hj) * P

## Key Files

| File                        | Description                                                                                 |
|:---------------------------:|:-------------------------------------------------------------------------------------------:|
| /src/main.cpp               | Source code for **main loop** that handles **uWebSockets communication to simulator**       |
| /src/FusionEKF.cpp, .h      | Source code for **sensor fusion algorithm** that processes RADAR/LIDAR measurements         |
| /src/kalman_filter.cpp, .h  | Source code for **standard Kalman filter** and **Extended Kalman filter** calculations      |
| /src/tools.cpp, .h          | Source code for calculating **Jacobian** matrix and **RMSE** values                         |
| /src/Eigen/                 | Eigen library for matrix calculations in C++                                                |
| /build/ExtendedKF           | Output **executable program binary**                                                        |
| install-mac.sh              | Script for Mac to install uWebSocketIO required to interface with simulator                 |
| install-ubuntu.sh           | Script for Linux to install uWebSocketIO required to interface with simulator               |

The original Udacity project repository is [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).

## How to Build and Run Code

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two scripts (**install-mac.sh** and **install-ubuntu.sh**) that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

If using Xcode to build, run the following commands:

1. mkdir xbuild
2. cd xbuild
3. cmake -G "Xcode" ..
4. Open "ExtendedKF.xcodeproj" in Xcode and build
5. cd Debug
6. ./ExtendedKF

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Communication protocol between uWebSocketIO and Simulator

**INPUT to main.cpp**: values provided by the simulator to the C++ program

* ["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)

**OUTPUT from main.cpp**: values provided by the C++ program to the simulator

* ["estimate_x"] <= kalman filter estimated position x
* ["estimate_y"] <= kalman filter estimated position y
* ["rmse_x"]
* ["rmse_y"]
* ["rmse_vx"]
* ["rmse_vy"]