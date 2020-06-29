# Go_Chase_It_ROS

## Aim

Driving a skid-steer robot in Gazebo 3D environment interfaced with ROS to chase any coloured ball with the help of hough circle transformation.

![Overview](https://github.com/aaryapatel007/Go_Chase_It_ROS/blob/master/video/chase_ball.gif)  

1. `drive_bot`:  
* Create a `my_robot` ROS package to hold your robot, the white ball, and the world.
* Design a skid-steer robot with the Unified Robot Description Format. Add two sensors to the robot: a lidar and a camera. Add Gazebo plugins for the skid-steer robot, lidar, and camera.
* House your robot inside the Gazebo 3D environment.
* Add a white-colored ball to your Gazebo world and save a new copy of this world.
* The `world.launch` file will launch the world with the white-colored ball and the robot.
2. `ball_chaser`:
* Create a `ball_chaser` ROS package to hold your C++ nodes.
* Write a `drive_bot` C++ node that will provide a `ball_chaser/command_robot` service to drive the robot by controlling its linear x and angular z velocities. The service will publish to the wheel joints and return back the requested velocities.
* Write a `process_image` C++ node that reads the robot’s camera image, analyzes it to determine the presence and position of a white ball using [Hough Circle Transform](https://docs.opencv.org/master/da/d53/tutorial_py_houghcircles.html). If a white ball exists in the image, the node will request a service via a client to drive the robot towards it.
* The `ball_chaser.launch` will run both the `drive_bot` and the `process_image` nodes.

## Prerequisites/Dependencies  

* Gazebo >= 7.0  
* ROS Kinetic  
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
  
## Project Description  

Directory Structure  
```
.Go_Chase_It_ROS                                   
├── catkin_ws                                  # Catkin workspace
│   ├── src
│   │   ├── ball_chaser                        # ball_chaser package        
│   │   │   ├── launch                         # launch folder for launch files
│   │   │   │   ├── ball_chaser.launch
│   │   │   ├── src                            # source folder for C++ scripts
│   │   │   │   ├── drive_bot.cpp
│   │   │   │   ├── process_images.cpp
│   │   │   ├── srv                            # service folder for ROS services
│   │   │   │   ├── DriveToTarget.srv
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
│   │   ├── my_robot                           # my_robot package        
│   │   │   ├── launch                         # launch folder for launch files   
│   │   │   │   ├── robot_description.launch
│   │   │   │   ├── world.launch
│   │   │   ├── meshes                         # meshes folder for sensors
│   │   │   │   ├── hokuyo.dae
│   │   │   ├── urdf                           # urdf folder for xarco files
│   │   │   │   ├── my_robot.gazebo
│   │   │   │   ├── my_robot.xacro
│   │   │   ├── worlds                         # world folder for world files
│   │   │   │   ├── myworld.world
│   │   │   ├── CMakeLists.txt                 # compiler instructions
│   │   │   ├── package.xml                    # package info
├── my_ball                                    # Model files 
│   ├── model.config
│   ├── model.sdf
├── pic                                     
│   ├── chase_ball.gif                          # video
```

## Run the project

* Clone this repository under the `src` folder of your `catkin_ws`.
* Open the repository and make  
```
cd /home/workspace/catkin_ws/
catkin_make
```
* Launch my_robot in Gazebo to load both the world and plugins  
```
roslaunch my_robot world.launch
```   
* Launch ball_chaser and process_image nodes  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```  
* Visualize  
```
cd /home/workspace/catkin_ws/
source devel/setup.bash
rosrun rqt_image_view rqt_image_view  
```  
## License

This repository is licensed under the terms of the MIT license.
