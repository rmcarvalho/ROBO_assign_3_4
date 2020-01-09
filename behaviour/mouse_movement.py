#!/usr/bin/env python

import os
from pynput import keyboard
import time
import rospy
import math
import copy
import numpy
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion


class MouseUtils(object):

    def __init__(self):

        self.check_all_sensors_ready()

        rospy.Subscriber("/mouseA/joint_states", JointState, self.joints_callback)
        rospy.Subscriber("/mouseA/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/mouseA/laser/cat_scan", LaserScan, self.laser_cat_callback)
        rospy.Subscriber("/mouseA/laser/wall_scan", LaserScan, self.laser_wall_callback)

        self._right_roll_vel = rospy.Publisher('/mouseA/right_diff_drive_controller/command', Float64, queue_size=1)
        self._left_roll_vel = rospy.Publisher('/mouseA/left_diff_drive_controller/command', Float64, queue_size=1)

        self.check_publishers_connection()

    def check_all_sensors_ready(self):
        self.disk_joints_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.disk_joints_data = rospy.wait_for_message("/mouseA/joint_states", JointState, timeout=1.0)
                rospy.loginfo("Current mouse/joint_states READY=>" + str(self.disk_joints_data))
            except:
                rospy.logerr("Current mouse/joint_states not ready yet, retrying for getting joint_states")

        self.mouse_odom_data = None
        while self.mouse_odom_data is None and not rospy.is_shutdown():
            try:
                self.mouse_odom_data = rospy.wait_for_message("/mouseA/odom", Odometry, timeout=1.0)
                rospy.loginfo("Current /mouseA/odom READY=>" + str(self.mouse_odom_data))
            except:
                rospy.logerr("Current /mouseA/odom not ready yet, retrying for getting odom")

        self.mouse_laser_wall_data = None
        while self.mouse_laser_wall_data is None and not rospy.is_shutdown():
            try:
                self.mouse_laser_wall_data = rospy.wait_for_message("/mouseA/laser/wall_scan", LaserScan, timeout=1.0)
                rospy.loginfo("Current /mouseA/laser/wall_scan READY=>" + str(self.mouse_laser_wall_data))
            except:
                rospy.logerr("Current /mouseA/laser/wall_scan not ready yet, retrying for getting laser scan")

        self.mouse_laser_cat_data = None
        while self.mouse_laser_cat_data is None and not rospy.is_shutdown():
            try:
                self.mouse_laser_cat_data = rospy.wait_for_message("/mouseA/laser/cat_scan", LaserScan, timeout=1.0)
                rospy.loginfo("Current /mouseA/laser/cat_scan READY=>" + str(self.mouse_laser_cat_data))
            except:
                rospy.logerr("Current /mouseA/laser/cat_scan not ready yet, retrying for getting laser scan")

        rospy.loginfo("ALL SENSORS READY")

    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._right_roll_vel.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.loginfo("No susbribers to _right_roll_vel yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        while (self._left_roll_vel.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.loginfo("No susbribers to _left_roll_vel yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass

        rospy.loginfo("_base_pub Publisher Connected")
        rospy.loginfo("All Publishers READY")

    def joints_callback(self, data):
        self.joints = data

    def odom_callback(self, data):
        self.odom = data

    def laser_wall_callback(self, data):
        self.laser_wall = data

    def laser_cat_callback(self, data):
        self.laser_cat = data

    def move_robot(self, linear_vel, angular_vel):
        wheel_dist = 0.2
        wheel_radius = 0.035

        right_wheel_ang_vel = (linear_vel + (angular_vel * (wheel_dist / 2.0))) / wheel_radius
        left_wheel_ang_vel = (linear_vel - (angular_vel * (wheel_dist / 2.0))) / wheel_radius

        self.move_joints(roll_speed_left=left_wheel_ang_vel, roll_speed_right=right_wheel_ang_vel)

    # Reinforcement Learning Utility Code
    def move_joints(self, roll_speed_left, roll_speed_right):

        left_joint_speed_value = Float64()
        right_joint_speed_value = Float64()
        left_joint_speed_value.data = roll_speed_left
        right_joint_speed_value.data = roll_speed_right

        self._right_roll_vel.publish(right_joint_speed_value)
        self._left_roll_vel.publish(left_joint_speed_value)

    def get_mouse_state(self):

        # We convert from quaternions to euler
        orientation_list = [self.odom.pose.pose.orientation.x,
                            self.odom.pose.pose.orientation.y,
                            self.odom.pose.pose.orientation.z,
                            self.odom.pose.pose.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        # We get the distance from the origin
        start_position = Point()
        start_position.x = 0.0
        start_position.y = 0.0
        start_position.z = 0.0

        distance = self.get_distance_from_point(start_position,
                                                self.odom.pose.pose.position)

        mouse_state = [
            round(self.joints.velocity[0], 1),
            round(distance, 1),
            round(roll, 1),
            round(pitch, 1),
            round(yaw, 1)
        ]

        return mouse_state

    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

def on_press(key):
    if key.char == 'w':
        keys[0] = True
    elif key.char == 'a':
        keys[1] = True
    elif key.char == 's':
        keys[2] = True
    elif key.char == 'd':
        keys[3] = True
    elif key.char == 'e':
        keys[4] = True
        return False

    return True


def on_release(key):
    if key.char == 'w':
        keys[0] = False
    elif key.char == 'a':
        keys[1] = False
    elif key.char == 's':
        keys[2] = False
    elif key.char == 'd':
        keys[3] = False
    elif key.char == 'e':
        keys[4] = False

    return True


def movement_controller():
    lin = 1.0
    ang = 1.0

    # w pressed
    if keys[0]:
        if keys[1]:
            mouse.move_robot(linear_vel=lin, angular_vel=ang)
        elif keys[3]:
            mouse.move_robot(linear_vel=lin, angular_vel=-ang)
        else:
            mouse.move_robot(linear_vel=lin, angular_vel=0.0)
    elif keys[2]:
        if keys[1]:
            mouse.move_robot(linear_vel=-lin, angular_vel=-ang)
        elif keys[3]:
            mouse.move_robot(linear_vel=-lin, angular_vel=ang)
        else:
            mouse.move_robot(linear_vel=-lin, angular_vel=0.0)
    elif keys[1]:
        mouse.move_robot(linear_vel=0.0, angular_vel=ang)
    elif keys[3]:
        mouse.move_robot(linear_vel=0.0, angular_vel=-ang)
    else:
        mouse.move_robot(linear_vel=0.0, angular_vel=0.0)

def clear_states(saved_state):
    global is_wandering, wander_rotation, is_running, run_rotation, is_following, follow_rotation
    if saved_state != 'follow':
        is_following = False
        follow_rotation = float("inf")
    if saved_state != 'wander':
        is_wandering = False
        wander_rotation = float("inf")
    if saved_state != 'run':
        is_running = False

#reduce angle to being between -pi and pi
def lowest_angle(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    while angle < math.pi:
        angle += 2*math.pi
    return angle

is_wandering = False
wander_rotation = float("inf")
def wander():
    print("Wandering...")
    global is_wandering, wander_rotation
    if is_running:
        is_wandering=True

    clear_states("wander")

    ang_vel = 0.0
    mouse_state = mouse.get_mouse_state()
    ang_y = mouse_state[3]
    if not is_wandering:
        is_wandering = True
        wander_rotation = random.uniform(-math.pi, math.pi) + ang_y

    angle_diff = (ang_y - wander_rotation) * 180 / math.pi

    if wander_rotation != float("inf") and angle_diff >= 10:
        ang_vel = 1.0 * angle_diff / abs(angle_diff)
    elif angle_diff < 10:
        wander_rotation = float("inf")

    mouse.move_robot(linear_vel=1.0, angular_vel=ang_vel)

is_running = False
def run_from_cat(cat_angles):
    print('Running From Cat')
    global is_running
    ang_vel = 0.0
    
    if not is_running:
        is_running = True

    clear_states("run")
    
    closest_cat = (0,float("inf"))
        
    for angle, distance in cat_angles:
        if distance < closest_cat[1]:
            closest_cat = (angle, distance)

    mouse_state = mouse.get_mouse_state()
    ang_y = mouse_state[3]

    angle_diff = (closest_cat[0] + math.pi) * 180 / math.pi
    ang_vel = 1.0 * angle_diff / abs(angle_diff)
    
    mouse.move_robot(linear_vel=1.0, angular_vel=ang_vel)

is_following = False
follow_rotation = float("inf")

def follow_wall(wall_angles):
    print('Following Wall')
    global is_following, follow_rotation
    
    clear_states("follow")

    closest_wall = (0,float("inf"))
        
    for angle, distance in wall_angles:
        if distance < closest_wall[1]:
            closest_wall = (angle, distance)
    
    alpha = math.pi/2 - closest_wall[0]

    ang_vel = (20 * math.sin(alpha) - (closest_wall[1]- 0))*0.5

    mouse.move_robot(linear_vel=1.0, angular_vel=ang_vel)


def decide():
    cat_angles, wall_angles = evaluate_lasers()

    # run if cat around, follow wall if any found or wander the map
    if len(cat_angles) > 0:
        run_from_cat(cat_angles)
    elif len(wall_angles) > 0:
        follow_wall(wall_angles)
    else:
        wander()


def evaluate_lasers():
    cat_angles = []
    wall_angles = []
    max_dif = -float("inf")
    for  i in range(len(mouse.laser_wall.ranges)):
        if mouse.laser_cat.ranges[i] >= mouse.laser_cat.range_max:
            continue


        current_angle = mouse.laser_cat.angle_min + i * mouse.laser_cat.angle_increment
        if mouse.laser_wall.ranges[i] >= mouse.laser_cat.ranges[i] - 0.06 and mouse.laser_wall.ranges[i] <= mouse.laser_cat.ranges[i] + 0.06:
            wall_angles.append((current_angle, mouse.laser_wall.ranges[i])) # wall detected
        else:
            print mouse.laser_cat.ranges[i] - mouse.laser_wall.ranges[i]
            cat_angles.append((current_angle, mouse.laser_cat.ranges[i])) # cat detected
    return cat_angles, wall_angles

# check detected obstacles

mouse = None

if __name__ == "__main__":
    # # mouse_systems_test()
    rospy.init_node('mouse_systems_test_node', anonymous=True, log_level=rospy.INFO)

    mouse = MouseUtils()
    
    rate = rospy.Rate(10)

    while True:
        decide()
        rate.sleep()
# #   KEYBOARD controlls
#     listener = keyboard.Listener(on_press=on_press, on_release=on_release)

#     # os.system("stty -echo")
#     listener.start()

#     rospy.loginfo('Started keyboard listener')

#     rate = rospy.Rate(10)

#     while not keys[4]:
#         movement_controller()
#         rate.sleep()

#     listener.join()
# os.system("stty echo")
