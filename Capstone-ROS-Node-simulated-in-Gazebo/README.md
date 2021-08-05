# Capstone - ROS Node simulated in Gazebo
This Capstone project about ROS Node simulated in Gazebo gathers the main principles learned throughout the Udacity Nanodegree Program called Become a C++ Developer. This project aims to demonstrate that i can independently create applications using a wide range of C++ features.

The ROS package of this project allows us to save the path that the robot has done from point "X" to point "B" in a `.png` image. This package was built and tested in Ubuntu 20.04.2 LTS, ROS Noetic, Gazebo 11, Boost 1.71 and OpenCV 4.2.

## Table of contents
* [Project file and class structure](#Project-file-and-class-structure)
* [Addressed capstone rubric points](#Addressed-capstone-rubric-points)
* [Dependencies](#Dependecies)
* [Requirements](#Requirements)
* [Build and run the project](#Build-and-run-the-project)
* [Expected behavior](#Expected-behavior)

### Project file and class structure
```sh
Capstone-ROS-Node-simulated-in-Gazebo
├── README.md
└── src
    └── capstone
        ├── CMakeLists.txt
        ├── include
        │   └── capstone
        │       ├── path_manager.h
        │       └── robot_pose_publisher.h
        ├── package.xml
        └── src
            ├── capstone_node.cpp
            ├── path_manager.cpp
            └── robot_pose_publisher.cpp
```

**`capstone_node.cpp`**

It runs two different ROS nodes (explained below) in different threads and properly manages the callback messages.

**`path_manager.h` and `path_manager.cpp`**

It saves the robot's path during the execution to eventually save it in an image if the user wants and when he or she wants.

It uses mutex when accessing the robot's pose to avoid other threads from accessing the same data and changing it at the same time.

For saving the path the robot did, this class also takes care of changing of coordinate systems, in this case from ROS world coordinate system to OpenCV coordinate system, and indicates with nice colours in the image the start point and endpoint of the robot.

**`robot_pose_publisher.h` and `robot_pose_publisher.cpp`**

This class gets the input from the keyboard commands typed by the user to teleoperate the robot in simulation. It computes the necessary transformations of the odom coordinate frame to the robot's base coordinate frame to get the robot's pose in ROS world coordinate system.

It also makes use of mutex when getting the keyboard commands, so that access to the data is safe.

### Addressed capstone rubric points
This application satisfies all criteria for the README (the current file you are reading) and Compiling and Testing. In this case, `catkin build` of `catkin_tools` has been used instead of `cmake` and `make`.

Additionally, it satisfies Loops, Functions, I/O. It writes data into an image and accepts input from the user to teleoperate the robot in simulation.

It also satisfies Object Oriented Programming using classes to group appropriately data and functions and public, protected and private attributes initialized through member initialization lists.

The Memory Management is taken into account as well, using references and pass-by-reference functions, taking advantage of the constructors and destructors to follow the Resource Acquisition Is Initialization (RAII) pattern, and shared smart pointers are used alongside ROS.

Regarding the concurrency, the project uses multiple threads in execution for spinning ROS callbacks. It also uses locks to protect data shared across multiple threads in the project code.

### Dependecies
* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)
* [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/installing.html) (Recommended)
* [Boost](https://www.howtoinstall.me/ubuntu/18-04/libboost-all-dev/)
* [OpenCV](https://learncybers.com/how-to-install-opencv-on-ubuntu-20-04/#Installing_OpenCV_from_the_Ubuntu_Repository)
* turtlebot3: gazebo and teleop

_This ROS node could work using different versions and distros but it has been tested in the ones mentioned at the top._

### Requirements
The following two debian packages have to be installed in order to be able to run the project:
```sh
sudo apt-get update
sudo apt-get install ros-noetic-turtlebot3-gazebo ros-noetic-turtlebot3-teleop
```
_Note that if you have a different ROS distribution installed you will have to change `noetic` to your corresponding one. This is applied to all the occurrences below._

### Build and run the project
Clone and build the project:
```sh
cd && git clone https://github.com/YueErro/udacity_cplusplus.git
. /opt/ros/noetic/setup.bash
cd udacity_cplusplus/Capstone-ROS-Node-simulated-in-Gazebo && catkin build
```

Run the project:
```sh
# Terminal 1: Launch turtlebot3 simulated robot in Gazebo
. /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

# Terminal 2: Run the ROS node
cd ~/udacity_cplusplus/Capstone-ROS-Node-simulated-in-Gazebo
. devel/setup.bash
rosrun capstone capstone_node

# Terminal 3: Teleoperate turtlebot3
. /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=burger
# Teleoperate the robot as you wish
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Terminal 4: Save turtlebot3's path
. /opt/ros/noetic/setup.bash
# Publish into the topic every time you want to save an image of the path that the robot did until that time
rostopic pub -1 /is_done std_msgs/Bool "data: true"
```

### Expected behavior
Launch the Gazebo world with the turtlebot3 and run my capstone ROS node. Once ready, launch the teleop ROS package in order to be able to teleoperate the robot in simulation.
Whenever you want, publish `true` as explained above, in the terminal 4, and the path that you commanded the robot to do will be saved in an image.

Each and every time it is published `true` into `/is_done` ROS topic, the image of the robot's path will be pop up and if you press any key it will be saved. Alternatively, if `false` is published, the path done by the robot will be removed and not saved in the next image.

The images of the path that the robot has done are saved at `/tmp` under `capstone_<data_time>.png` name.
There is a demo video that can be downloaded from [here](https://github.com/YueErro/udacity_cplusplus/raw/master/Capstone-ROS-Node-simulated-in-Gazebo/demo.mp4).
