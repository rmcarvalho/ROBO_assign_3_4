# ROBO_assign_3_4

## Operative System

Ubuntu 18.04

## ROS Version

ROS Melodic 

## Dependencies

ROS Melodic

Catkin Workspace

Gazebo

gazebo_ros package

ros_state_publisher



## Directory Organization
    .
    ├── mouse_description           # Compiled files (alternatively `dist`)
    |    ├── config                 # End-to-end, integration tests (alternatively `e2e`)
    |    ├── launch                 # Launch Files
    |    ├── meshes                 # Mouse mesh
    |    ├── models                 # Arena Models
    |    ├── scripts                # All scripts
    |    ├── urdf                   # Robot definition
    |    └── worlds                 # Arena world definition
    └── spawn_robot_tools_pkg  


## To run:

Starts world with robots:
```
$ roslaunch mouse_description main.launch
```

## Moving the robot
```
rostopic pub /mouse/left_front_diff_drive_controller/command std_msgs/Float64 "data: 5.0"
rostopic pub /mouse/right_front_diff_drive_controller/command std_msgs/Float64 "data: 5.0"
```

To run with a second robot, drag the package of that robot to the catkin's workspace.

## Authors

Ricardo Carvalho - up201503717

Vítor Magalhães - up201503447

## Course

Master in Informatics and Computing Engineering

Faculty of Engineering of the University of Porto 
