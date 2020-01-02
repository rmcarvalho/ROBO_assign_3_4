# ROBO_assign_3_4

## To run:
```
$ sudo cp -i src/mouse_description/worlds/arena.xml /usr/share/gazebo-7/worlds
$ roslaunch mouse_description start_world.launch
```

```
$ roslaunch mouse_description put_robot_in_world.launch 
```

## Moving the robot
```
rostopic pub /mouse/left_front_diff_drive_controller/command std_msgs/Float64 "data: 5.0"
rostopic pub /mouse/right_front_diff_drive_controller/command std_msgs/Float64 "data: 5.0"
```
