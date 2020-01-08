#!/usr/bin/env python

import os
from pynput import keyboard
import time
import rospy
import math
import copy
import numpy
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
        # rospy.Subscriber("/mouse/laser/scan", LaserScan, self.laser_callback)

        self._right_roll_vel = rospy.Publisher('/mouseA/right_diff_drive_controller/command', Float64, queue_size=1)
        self._left_roll_vel = rospy.Publisher('/mouseA/left_diff_drive_controller/command', Float64, queue_size=1)

        self.check_publishers_connection()

    def check_all_sensors_ready(self):
        self.disk_joints_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.disk_joints_data = rospy.wait_for_message("/mouseA/joint_states", JointState, timeout=1.0)
                rospy.loginfo("Current mouse/joint_states READY=>"+str(self.disk_joints_data))

            except:
                rospy.logerr("Current mouse/joint_states not ready yet, retrying for getting joint_states")

        self.mouse_odom_data = None
        while self.mouse_odom_data is None and not rospy.is_shutdown():
            try:
                self.mouse_odom_data = rospy.wait_for_message("/mouseA/odom", Odometry, timeout=1.0)
                rospy.loginfo("Current /mouseA/odom READY=>" + str(self.mouse_odom_data))

            except:
                rospy.logerr("Current /mouseA/odom not ready yet, retrying for getting odom")

        # self.mouse_laser_data = None
        # while self.mouse_laser_data is None and not rospy.is_shutdown():
        #     try:
        #         self.mouse_laser_data = rospy.wait_for_message("/mouse/laser/scan", LaserScan, timeout=1.0)
        #         rospy.loginfo("Current /mouse/laser/scan READY=>" + str(self.mouse_laser_data))

        #     except:
        #         rospy.logerr("Current /mouse/laser/scan not ready yet, retrying for getting laser scan")

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

    def laser_callback(self, data):
        self.laser = data

    def move_robot(self, linear_vel, angular_vel):
        wheel_dist = 0.4
        wheel_radius = 0.08

        right_wheel_ang_vel = (linear_vel + (angular_vel*(wheel_dist/2.0)))/wheel_radius
        left_wheel_ang_vel = (linear_vel - (angular_vel*(wheel_dist/2.0)))/wheel_radius

        self.move_joints(roll_speed_left=left_wheel_ang_vel, roll_speed_right=right_wheel_ang_vel)

    # Reinforcement Learning Utility Code
    def move_joints(self, roll_speed_left, roll_speed_right):

        left_joint_speed_value = Float64()
        right_joint_speed_value = Float64()
        left_joint_speed_value.data = roll_speed_left
        right_joint_speed_value.data = roll_speed_right

        #rospy.loginfo("Left Wheel Velocity>>"+str(left_joint_speed_value))
        #rospy.loginfo("Right Wheel Velocity>>"+str(right_joint_speed_value))
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
                        round(self.joints.velocity[0],1),
                        round(distance,1),
                        round(roll,1),
                        round(pitch,1),
                        round(yaw,1)
                    ]

        return mouse_state

    def observation_checks(self, mouse_state):
        # Maximum distance to travel permited in meters from origin
        max_distance=10.0

        if (mouse_state[1] > max_distance):
            rospy.logerr("Mouse Too Far==>"+str(mouse_state[1]))
            done = True
        else:
            rospy.loginfo("Mouse NOT Too Far==>"+str(mouse_state[1]))
            done = False

        return done

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

    def get_reward_for_observations(self, state):

        # We reward it for lower speeds and distance traveled

        speed = state[0]
        distance = state[1]

        # Positive Reinforcement
        reward_distance = distance * 10.0
        # Negative Reinforcement for magnitude of speed
        reward_for_efective_movement = -1 * abs(speed)

        reward = reward_distance + reward_for_efective_movement

        rospy.loginfo("Reward_distance="+str(reward_distance))
        rospy.loginfo("Reward_for_efective_movement= "+str(reward_for_efective_movement))

        return reward

def mouse_systems_test():
    rospy.init_node('mouse_systems_test_node', anonymous=True, log_level=rospy.INFO)

    mouse_utils_object = MouseUtils()

    rospy.loginfo("Moving to Linear==>1.0 Angular==>0.0")
    mouse_utils_object.move_robot(linear_vel=0.5, angular_vel=0.0)
    time.sleep(4)

    rospy.loginfo("Stopping")
    mouse_utils_object.move_robot(linear_vel=0.0, angular_vel=0.0)
    time.sleep(4)

    rospy.loginfo("Moving to Linear==>0 Angular==>-1.0")
    mouse_utils_object.move_robot(linear_vel=0.0, angular_vel=-1.0)
    time.sleep(4)

    rospy.loginfo("Stoping")
    mouse_utils_object.move_robot(linear_vel=0.0, angular_vel=0.0)
    time.sleep(4)

    rospy.loginfo("Moving to Linear==>-0.2 Angular==>0")
    mouse_utils_object.move_robot(linear_vel=-0.2, angular_vel=0.0)
    time.sleep(4)

    rospy.loginfo("Stoping")
    mouse_utils_object.move_robot(linear_vel=0.0, angular_vel=0.0)
    time.sleep(4)

    mouse_state = mouse_utils_object.get_mouse_state()
    done = mouse_utils_object.observation_checks(mouse_state)
    reward = mouse_utils_object.get_reward_for_observations(mouse_state)

    rospy.loginfo("Done==>"+str(done))
    rospy.loginfo("Reward==>"+str(reward))

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


mouse = None
# w, a, s, d, e
keys = [False, False, False, False, False]

if __name__ == "__main__":
    # mouse_systems_test()
    rospy.init_node('mouse_systems_test_node', anonymous=True, log_level=rospy.INFO)

    mouse = MouseUtils()

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)

    os.system("stty -echo")
    listener.start()

    rospy.loginfo('Started keyboard listener')

    rate = rospy.Rate(10)

    while not keys[4]:
        movement_controller()
        rate.sleep()

    listener.join()
    os.system("stty echo")