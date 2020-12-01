# Build My World

## Project Summary
1. Build a single floor wall structure using the Building Editor tool in Gazebo. Apply at least one feature, one color, and optionally one texture to your structure
2. Make sure there's enough space between the walls for a robot to navigate.
3. Model any object of your choice using the Model Editor tool in Gazebo. Your model links should be connected with joints.
4. Import your structure and two instances of your model inside an empty Gazebo World.
5. Import at least one model from the Gazebo online library and implement it in your existing Gazebo world.
6. Write a C++ World Plugin to interact with your world. Your code should display “Welcome to ’s World!” message as soon as you launch the Gazebo world file.

## Folder Contents
* [build_my_world](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/tree/main/build_my_world)
    * **model** - It contains model files for :
        * **my_building** - A single floor structure designed in the Building Editor tool of Gazebo
        * **skid_steer_bot** - A robot designed in the Model Editor tool of Gazebo
    * **script** - It contains plugin codes for :
        * **welcome_message.cpp** - Gazebo world plugin C++ script to display welcome message.
        * **move_robot_angular.cpp** - Gazebo world plugin C++ script to give angular velocity to robot making it rotate at it's own place.
    * **world** - It contains world file :
        * **my_world.world** - Gazebo world file that includes the models as asked in project.
    * **CMakeLists.txt** - File to link the C++ code to libraries and build it.

## Screenshot
* Gazebo World
![picture alt](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/tree/main/build_my_world/images/gazebo_world.jpg "Gazebo World")
* Skid Steer Robot
![picture alt](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/tree/main/build_my_world/images/skid_steer_robot.jpg "Skid Steer Robot")

## Build and Run
```
cd ~/
git clone https://github.com/prasun2712/udacity_robotics_nano_degree_submissions.git
cd ~/udacity_robotics_nano_degree_submissions/build_my_world
mkdir build
cd build
cmake ..
make
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/udacity_robotics_nano_degree_submissions/build_my_world/build
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/udacity_robotics_nano_degree_submissions/build_my_world/model
cd ../world
gazebo my_world.world
```
