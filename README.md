# ROBO_assign_3_4

## To run:
```
roslaunch gazebo_ros empty_world.launch
```

```
roslaunch mouse_description put_robot_in_world.launch 
```

## Moving the robot
```
rostopic pub /mouse/left_front_diff_drive_controller/command std_msgs/Float64data: 5.0"
rostopic pub /mouse/right_front_diff_drive_controller/command std_msgs/Float64data: 5.0"
```
