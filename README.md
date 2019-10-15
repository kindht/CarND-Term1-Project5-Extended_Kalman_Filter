# 无人驾驶(纳米学位)-项目5(Term1 Final Project)-卡尔曼滤波
# CarND-Project5-Extended Kalman Filter  (C++)

[//]: # (Image References)
[image1.1]: ./examples/example.gif
[image1.2]: ./examples/example.png
[image2]: ./Pass-certificate.png 

This is the 5th project(i.e final project) for the **Self-Driving Car Engineer** Nanodegree program at [Udacity](https://cn.udacity.com/course/self-driving-car-engineer--nd013)

In this project I implemented a **kalman filter** in C++ to estimate the state of a moving object of interest with noisy lidar and radar measurements. 

The steps of this project are the following: 

* **initializing** Kalman filter variables
* **predicting** where our object is going to be after a time step Δt
* **updating** where our object is based on sensor measurements
* **repeating** the prediction and update steps in a **loop*
* Report out the **RMSE** (root mean squared error) of the estimations

 The project is implemented in **C++**, source files are in [.src](./src) . The source code was compiled and running on MacOS 10.14.5
 
---
#### Example output 
**RMSE** values obtained are as the following , first image shows the simulator interface 

[//]: # (Image References)

[image0.1]:   ./Result/Result-simu-1.png "Simulator"
[image0.2]:   ./Result/Result-data.png "Local"
[image0.3]:   ./Result/Result-simu-2.png "Simulator"

[image1.1.1]: ./Result/Test-Laser-Only-simu1.png "Simu1"
[image1.1.2]: ./Result/Test-Laser-Only-simu2.png "Simu2"
[image1.1]:   ./Result/Test-Laser-Only-data.png "Laser"
[image1.2]:   ./Result/Test-Radar-Only-data.png "Rader "

Here is the result running with the **simulator** with **Dataset 1**:  

![alt text][image0.1]

Here is the reuslt running with test data on **local machine** (MacOS 10.14.5)

![alt text][image0.2]

Here is the result running with the **simulator** with **Dataset 2**:  

![alt text][image0.3]

&nbsp;
## Test with Laser Only  

I tested to use **Laser Only** for **Dataset 1**, the result is just **a little bit worse** than fusion of 2 sensors, this shows that the **Laser sensor is highly effective**.  

![alt text][image1.1.1]  

Here is the result testing on simulator with  **Laser Only** for **Dataset 2**, similarly to performance with Dataset1  

![alt text][image1.1.2]

## Compare : Laser Only vs. Radar Only
Here is the result using **Laser Only** with **Data in the data file** (same as Dataset1 in simulator)  

![alt text][image1.1] 


Here is the result testing with  **Radar Only** for **Data in the data file**, it is obviously worse than laser only, shows that only with Radar


![alt text][image1.2] 

### Conclusion:
we can hardly make good estimations with Radar only, laser sensor will still be necessary.

&nbsp;
## Basic Build Instructions
The main program can be built and run by doing the following from the project top directory: 

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

The project directory includes files in **src/**:   
- FusionEKF.cpp, FusionEKF.h,  
- kalman_filter.cpp, kalman_filter.h,  
- tools.cpp, and tools.h

Other files include: measurement_package.h, json.hpp

The main.cpp is for build and runing with simulator.

## Test with data file 
I created a **main_test.cpp** for running with data file for testing purpose.  (in directory of backup/)

---

## Important Dependencies
I used MacOS 10.14.5 with following versions of tools

* cmake = 3.15
* make = 3.81 
* g++ = 4.2.1
 




This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).


## Other References
* cmake = 3.15
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make = 3.81 
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* g++ = 4.2.1
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
