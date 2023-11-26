# mini_roomba
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

A basic walker algorithm implementation of roomba in ROS2 Humble.

## Dependencies

- **Ubuntu 22.04**
- **ROS2 Humble**  
- **Turtlebot3 ROS Package**

## Dependencies Setup
1. All turtlebot3 pakages install
    ```bash
    sudo apt install ros-humble-turtlebot3*
    ```
2. Humble and Gazebo ROS packages
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs 
    ```
3. Set up gazebo model path 
    ```bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
    prefix turtlebot3_gazebo \
    `/share/turtlebot3_gazebo/models/
    ```
## Build your package
```bash
mkdir -p mini_roomba/src
cd mini_roomba/src
git clone https://github.com/MayankD409/mini_roomba
cd mini_roomba
colcon build
```
## Launch 
```
cd mini_roomba/ros2_ws
source  install/setup.bash
ros2 launch mini_roomba walker.launch.py record_bag:=true
```
## ros bag recording 
```
ros2 bag info my_bag
```

## Run cppcheck and cpplint

For cppcheck (Execute from root of the package)
```bash
cppcheck --enable=all --std=c++11 --std=c++17 --enable=information --check-config --suppress=missingInclude --suppress=*:*test*/ --suppress=unmatchedSuppression $( find . -name *.cpp | grep -vE -e "^./build/")
```
For cpplint (Execute from root of the package)
```bash
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp
`````