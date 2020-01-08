# ROBO_assign_3_4

## To run:
```

Starts world with robots:
```
$ roslaunch mouse_description main.launch
```

## Moving the robot
```
rostopic pub /mouse/left_front_diff_drive_controller/command std_msgs/Float64 "data: 5.0"
rostopic pub /mouse/right_front_diff_drive_controller/command std_msgs/Float64 "data: 5.0"
```

With AI:

```
roslaunch mouse_training start_training.launch
```
