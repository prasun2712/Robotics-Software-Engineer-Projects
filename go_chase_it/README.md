# Go Chase It!

## Project Summary
In this project, you should create two ROS packages inside your `catkin_ws/src`: the `drive_bot` and the `ball_chaser`. Here are the steps to design the robot, house it inside your world, and program it to chase white-colored balls:

1. `drive_bot`:

    * Create a `my_robot` ROS package to hold your robot, the white ball, and the world.
    * Design a differential drive robot with the Unified Robot Description Format. Add two sensors to your robot: a lidar and a camera. Add Gazebo plugins for your robot’s differential drive, lidar, and camera. The robot you design should be significantly different from the one presented in the project lesson. Implement significant changes such as adjusting the color, wheel radius, and chassis dimensions. Or completely redesign the robot model! After all you want to impress your future employers :-D
    * House your robot inside the world you built in the ***Build My World*** project.
    * Add a white-colored ball to your Gazebo world and save a new copy of this world.
    * The `world.launch` file should launch your world with the white-colored ball and your robot.

2. `ball_chaser`:

    * Create a `ball_chaser` ROS package to hold your C++ nodes.
    * Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service should publish to the wheel joints and return back the requested velocities.
    * Write a `process_image` C++ node that reads your robot’s camera image, analyzes it to determine the presence and position of a white ball. If a white ball exists in the image, your node should request a service via a client to drive the robot towards it.
    * The `ball_chaser.launch` should run both the `drive_bot` and the `process_image` nodes.

## Folder Contents
* [go_chase_it](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/tree/main/go_chase_it)
    * **catkin_ws** - Workspace folder :
        * **src** - It contains the packages for the submission
            * **my_robot** - It contains the robot, a white ball and the world file.
                * **launch** :
                    * **skid_steer_diff_bot_base_laser.launch** - Launch file to load robot description and publish robot state and joint states.
                    * **world.launch** - Launch file to bring up the world with white ball in it and spawn the robot.
                * **meshes** : 3D mesh files.
                * **urdf** : Robot's xacro files
                * **world** : World file with white ball in it.
                * `CMakeLists.txt`
                * `package.xml`
            * **ball_chaser** - It contains the ROS nodes to detect white ball and drive the robot.
                * **launch** :
                    * **ball_chaser.launch** - Launch file to bringup the ros nodes, `drive_bot` and `process_image`
                * **src** - It contains two cpp files :
                    * **drive_bot.cpp** - To drive the robot with the help of service `ball_chaser/command_robot`.
                    * **process_image.cpp** - To detect white ball in the camera image and drive the robot by calling the service mentioned. 
                * **srv** - It contains the service file `DriveToTarget.srv`.
                * `CMakeLists.txt`
                * `package.xml`

## Screenshot
* Gazebo World with the **Robot** and the **White Ball**
![](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/blob/main/go_chase_it/videos_and_pictures/world.png "Gazebo World with the Robot and the White Ball")
* **Skid Steer Robot** with lidar and camera and also gazebo plugin for robot’s differential drive, lidar, and camera.
![](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/blob/main/go_chase_it/videos_and_pictures/robot.png "Skid Steer Robot with lidar and camera and also gazebo plugin for robot’s differential drive, lidar, and camera.")
* **White Ball**
![](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/blob/main/go_chase_it/videos_and_pictures/white_ball.png "White Ball")

## Build and Run
```
cd ~/
git clone https://github.com/prasun2712/udacity_robotics_nano_degree_submissions.git
cd ~/udacity_robotics_nano_degree_submissions/go_chase_it/catkin_ws
catkin_make
```
***Terminal 1***
```
roslaunch my_robot world.launch
```
***Terminal 2***
```
roslaunch ball_chaser ball_chaser.launch
```

## Demo Video
![](https://github.com/prasun2712/udacity_robotics_nano_degree_submissions/blob/main/go_chase_it/videos_and_pictures/demo_video.mp4 "Demo Video")