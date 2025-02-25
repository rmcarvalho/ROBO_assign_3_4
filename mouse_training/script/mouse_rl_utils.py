#!/usr/bin/env python

import time
import rospy
import math
import copy
import numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from tf.transformations import euler_from_quaternion

class MouseRLUtils(object):

    def __init__(self):

        self.check_all_sensors_ready()
        
        rospy.Subscriber("/mouse/joint_states", JointState, self.joints_callback)
        rospy.Subscriber("/mouse/odom", Odometry, self.odom_callback)

        self._roll_vel_pub = rospy.Publisher('/mouse/right_front_diff_drive_controller/command', Float64, queue_size=1)
        self.check_publishers_connection()
        
    
    def check_all_sensors_ready(self):
        self.disk_joints_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.disk_joints_data = rospy.wait_for_message("/mouse/joint_states", JointState, timeout=1.0)
                rospy.loginfo("Current mouse/joint_states READY=>"+str(self.disk_joints_data))

            except:
                rospy.logerr("Current mouse/joint_states not ready yet, retrying for getting joint_states")

        self.mouse_odom_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.mouse_odom_data = rospy.wait_for_message("/mouse/odom", Odometry, timeout=1.0)
                rospy.loginfo("Current /mouse/odom READY=>" + str(self.mouse_odom_data))

            except:
                rospy.logerr("Current /mouse/odom not ready yet, retrying for getting odom")
        rospy.loginfo("ALL SENSORS READY")
        
    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._roll_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.loginfo("No susbribers to _roll_vel_pub yet so we wait and try again")
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

    
    # Reinforcement Learning Utility Code
    def move_joints(self, roll_speed):
        
        joint_speed_value = Float64()
        joint_speed_value.data = roll_speed
        rospy.loginfo("Single Disk Roll Velocity>>"+str(joint_speed_value))
        self._roll_vel_pub.publish(joint_speed_value)
        
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
        
        
        # MAximum distance to travel permited in meters from origin
        max_distance=2.0
        
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



def mouse_rl_systems_test():
    
    rospy.init_node('mouse_rl_systems_test_node', anonymous=True, log_level=rospy.INFO)
    mouse_rl_utils_object = MouseRLUtils()
    
    rospy.loginfo("Moving to Speed==>80")
    mouse_rl_utils_object.move_joints(roll_speed=80.0)
    time.sleep(2)
    rospy.loginfo("Moving to Speed==>-80")
    mouse_rl_utils_object.move_joints(roll_speed=-80.0)
    time.sleep(2)
    rospy.loginfo("Moving to Speed==>0.0")
    mouse_rl_utils_object.move_joints(roll_speed=0.0)
    time.sleep(2)
    
    mouse_state = mouse_rl_utils_object.get_mouse_state()
    done = mouse_rl_utils_object.observation_checks(mouse_state)
    reward = mouse_rl_utils_object.get_reward_for_observations(mouse_state)
    
    rospy.loginfo("Done==>"+str(done))
    rospy.loginfo("Reward==>"+str(reward))
    

if __name__ == "__main__":
    mouse_rl_systems_test()