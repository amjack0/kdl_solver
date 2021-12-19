## kdl_solver

This repo uses `KDL` to solve the kinematics of the robot. 

## Dependencies and Library

`kdl_parser` https://github.com/ros/kdl_parser.git \
`orocos_kdl` https://github.com/orocos/orocos_kinematics_dynamics.git

## Build instructions

* git clone the repo and change directory with `cd kdl_solver`
* create a `build` folder with `mkdir build && cd build`
* run `cmake ..` and `make`
* if you have installed `kdl_parser` and `orocos_kdl` it should build successfully.

## Run

* Inside build directory `cd build` 
* run with `./kdl_solver`
