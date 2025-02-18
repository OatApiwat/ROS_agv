#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import csv
import os
import time

# Path to the CSV file
csv_file_path = '/home/ros3/ros1_ws/src/agv_pkg/src/dist_batt_reccord2.csv'
global count
count = 0
# Check if the file exists, if not create it and write the header
if not os.path.exists(csv_file_path):
    with open(csv_file_path, mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(['timestamp', 'distance', 'battery'])

# Initialize global variables to store the latest data
latest_distance = None
latest_battery = None

def distance_callback(data):
    global latest_distance
    latest_distance = data.data
    record_data()

def battery_callback(data):
    global latest_battery
    latest_battery = data.data

def record_data():
    global count
    count = count+1
    if latest_distance is not None and latest_battery is not None:
        timestamp = rospy.get_time()
        with open(csv_file_path, mode='a') as file:
            writer = csv.writer(file)
            writer.writerow([count, latest_distance, latest_battery])

def publish_cmd_vel():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()
    stop_cmd = Twist()

    move_cmd.linear.x = 0.28947
    move_cmd.linear.y = 0.0
    move_cmd.linear.z = 0.0
    move_cmd.angular.x = 0.0
    move_cmd.angular.y = 0.0
    move_cmd.angular.z = 0.0

    stop_cmd.linear.x = 0.0
    stop_cmd.linear.y = 0.0
    stop_cmd.linear.z = 0.0
    stop_cmd.angular.x = 0.0
    stop_cmd.angular.y = 0.0
    stop_cmd.angular.z = 0.0

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        start_time = time.time()
        while time.time() - start_time < 165:
            pub.publish(move_cmd)
            rate.sleep()
        start_time = time.time()
        while time.time() - start_time < 10:
            pub.publish(stop_cmd)
            rate.sleep()

def listener():
    rospy.init_node('dist_batt_listener', anonymous=True)
    rospy.Subscriber('/distance_topic', Float32, distance_callback)
    rospy.Subscriber('/battery_topic', Float32, battery_callback)
    publish_cmd_vel()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
