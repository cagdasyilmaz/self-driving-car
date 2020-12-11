# self-driving-car-lane-changing-cpp-and-matlab

Welcome to the self-driving-car-lane-changing-cpp-and-matlab wiki!

In this repository, Nonlinear Model Predictive Controller (NMPC) which is examined in the article "Model Predictive Cont"MPC-Based Approach to Active Steering for Autonomous Vehicle Systems" from F. Borrelli et al. is utilized for lane change maneuvering using active steering control. 

The proposed NMPC is employed via both MATLAB (R2018 Linux) and c++14 (gcc/g++ >= 9.3.0).  

# Dependencies

* **Simulation via MATLAB & YALMIP (For this simulation MATLAB should be installed on your computer!)**
1. Find a suitable folder for yalmip installation and create a folder, go to the folder in the MATLAB command window
2. Download YALMIP from website: https://yalmip.github.io/download/ and copy this .zip file to the folder above
3. Copy and past the following 3 lines MATLAB command window

>> unzip('YALMIP-master.zip','yalmip')

>> addpath(genpath([pwd filesep 'yalmip']));

>> savepath

4. Test YALMIP: run the following command in MATLAB command window, check available optimization tools among lists
> yalmiptest

* **Simulation Results via c++ (IPOPT, cppAD, matplotlibcpp)**

_Dependencies For C++ Implementation_
1. cmake >= 3.5 (Please change CMakeList.txt file for other cmake releases)
2. gcc/g++ >= 9.3.0 (other releases can be tried!)
3. Ipopt and CppAD: Please refer to this document, https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md, for installation instructions.
4. Eigen Library, http://eigen.tuxfamily.org/index.php?title=Main_Page, This is already part of the source codes

_Basic Build Instructions_
1. Make a build directory: **mkdir build && cd build**
2. Compile: **cmake .. && make**
3. Run it: **./active steering mpc**
