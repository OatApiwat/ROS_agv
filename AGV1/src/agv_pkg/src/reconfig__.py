#!/usr/bin/env python3

import rospy
from rosgraph_msgs.msg import Log
from dynamic_reconfigure.client import Client
from geometry_msgs.msg import Twist

count = 0

def reconfigure_callback(config):
    pass

def callback(data):
    global count
    if data.name == "/move_base":
        client = Client("/move_base/DWAPlannerROS", timeout=30)
        config = client.get_configuration()
        min_vel_x = config["min_vel_x"]
        rospy.loginfo("Received min_vel_x: {}".format(min_vel_x))
        if data.level == rospy.WARN or data.level == rospy.ERROR:
            rospy.logwarn("Received warning: {}".format(data.msg))
            if min_vel_x != -0.2:
                new_config = {
                    "min_vel_x": -0.2,
                    "min_vel_trans": -0.2
                }
                client.update_configuration(new_config)
                rospy.loginfo("Updated by WARNING")
            count = 0
        elif data.level == rospy.INFO:
            count += 1
            if min_vel_x == -0.2 and 20 <= count <= 21:
                new_config = {
                    "min_vel_x": 0.1,
                    "min_vel_trans": 0.1
                }
                client.update_configuration(new_config)
                rospy.loginfo("Updated by INFO")
                rospy.loginfo("Received info: {}".format(data.msg))
                count = 0

# def callback_cmd(data):
#     client = Client("/amcl", timeout=30)
#     config = client.get_configuration()
#     update_min_d = config["update_min_d"]
#     update_min_a = config["update_min_a"]
#     if data.linear.x <= 0.125 and data.angular.z <= 0.125:
#         if update_min_d != 0.05 or update_min_a != 0.05:
#             new_config = {
#                 "update_min_d": 0.05,
#                 "update_min_a": 0.05
#             }
#             client.update_configuration(new_config)
#     elif data.linear.x > 0.125 and data.angular.z > 0.125:
#         if update_min_d != 0.2 or update_min_a != 0.2:
#             new_config = {
#                 "update_min_d": 0.2,
#                 "update_min_a": 0.2
#             }
#             client.update_configuration(new_config)

def listener():
    rospy.init_node('error_warn_listener')
    rospy.Subscriber("/rosout", Log, callback)
    # rospy.Subscriber("/cmd_vel", Twist, callback_cmd)
    rospy.spin()

if __name__ == '__main__':
    listener()
