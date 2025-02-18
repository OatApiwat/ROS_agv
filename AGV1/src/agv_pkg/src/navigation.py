#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
import paho.mqtt.subscribe as subscribe
import paho.mqtt.client as mqtt
import mysql.connector
import time
from datetime import datetime
import signal
import sys
from math import atan2, radians

#-----------------------------------------------Setting for conection-----------------------------------------------#
#-------------------------ROS Parameter
global move_base, client, connection, start_point, finish_point, start_time, finish_time, duration, result_stat, goal_point, goal_x, goal_y, goal_th
start_point = "Home"
rospy.init_node('nav_goal', anonymous=False)
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
rate = rospy.Rate(5)  # 5 hz
rospy.loginfo("wait for the action server to come up")
move_base.wait_for_server(rospy.Duration(5))
time_out = 120  # time out if AGV cannot reach goal
#-------------------------mqtt connection
host = '192.168.0.167'
port = 1883
topic = '/ros_mqtt'
client = mqtt.Client()
try:
    client.connect(host, port)
    rospy.loginfo("Success connected to MQTT broker")
except ConnectionRefusedError:
    rospy.logwarn("Failed to connect to MQTT broker")
    exit()

#----------------------------Connect mysql
try:
    # เชื่อมต่อฐานข้อมูล MySQL
    connection = mysql.connector.connect(
        host='localhost',
        user='agv_mysql',
        password='123qwe',
        database='AGV_db'
    )
    if connection.is_connected():
        rospy.loginfo("Successfully connected to MySQL database")
    else:
        rospy.logwarn("Failed to connect to MySQL database")
        exit()  # หรือ sys.exit() สำหรับการจบโปรแกรม
except mysql.connector.Error as err:
    rospy.logwarn("Failed to connect to MySQL database: {}".format(err))
    exit()  # หรือ sys.exit() สำหรับการจบโปรแกรม
cursor = connection.cursor()
cursor.close()

#-----------------------------------------------------------Spacial Function-------------------------------------------------------#
# กำหนด handler สำหรับ SIGINT
def signal_handler(sig, frame):
    rospy.loginfo('Exiting the program...')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

#-----------------------------------------------Main Function for Navigation-----------------------------------------------#
def read_mqtt():
    global goal_point  # เพิ่มการประกาศตัวแปร global
    rospy.loginfo("Waiting for subscrite")
    msg = subscribe.simple(topics=str(topic), hostname=host, port=port)
    goal_point = msg.payload.decode("utf-8", "strict")
    rospy.loginfo("Subscrite Goal is : {}".format(goal_point))  # แก้ไขการพิมพ์ข้อความให้ถูกต้อง

def check_database():
    global goal_x, goal_y, goal_th  # เพิ่มการประกาศตัวแปร global
    rospy.loginfo("Checking information in database")
    cursor = connection.cursor()
    select_query = """
        SELECT goal_x, goal_y, goal_th FROM goal_tb WHERE goal_point = %s
    """
    cursor.execute(select_query, (goal_point,))
    result = cursor.fetchone()
    if result:
        goal_x, goal_y, goal_th = result
        rospy.loginfo("Found goal information in database - X: {}, Y: {}, Theta: {}".format(goal_x, goal_y, goal_th))
    else:
        rospy.logwarn("Goal information not found in database for goal point: {}".format(goal_point))
        read_mqtt()
    cursor.close()

def amcl_pose_callback(data):
    direction = atan2(data.pose.pose.x,data.pose.pose.y,goal_x,goal_y)
    direction_radians = radians(direction)
    q = quaternion_from_euler(0, 0, direction_radians)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(data.pose.pose.x, data.pose.pose.y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
    move_base.send_goal(goal)
    wait = move_base.wait_for_result(rospy.Duration(20))
    state = move_base.get_state()
    if wait and state == GoalStatus.SUCCEEDED:
        movebase_client()
        save_database()
    else:
        move_base.cancel_goal()

def rotate_direction():
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_pose_callback)

def movebase_client():
    global start_time, finish_time, finish_point, result_stat, duration  # เพิ่มการประกาศตัวแปร global
    q = quaternion_from_euler(0, 0, goal_th)
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(goal_x, goal_y, 0.000), Quaternion(q[0], q[1], q[2], q[3]))
    start_time = datetime.now()
    start_time_millis = time.time()
    move_base.send_goal(goal)
    rospy.loginfo("Going to goal: {}".format(goal_point))  # แก้ไขการพิมพ์ข้อความให้ถูกต้อง
    wait = move_base.wait_for_result(rospy.Duration(time_out))
    state = move_base.get_state()
    if wait and state == GoalStatus.SUCCEEDED:
        result_stat = "OK"
        finish_time = datetime.now()
        finish_point = goal_point
        duration = int(round(time.time() - start_time_millis))
        client.publish('/agv_feedback', 'Succeeded')
    else:
        move_base.cancel_goal()
        result_stat = "Fail"
        finish_time = datetime.now()
        finish_point = goal_point
        duration = int(round(time.time() - start_time_millis))
    rospy.loginfo(["Done result : " + result_stat])

def save_database():
    global start_point  # เพิ่มการประกาศตัวแปร global
    rospy.loginfo("Saving information in database")
    cursor = connection.cursor()
    dates = datetime.now().strftime("%Y-%m-%d") 
    times = datetime.now().strftime("%H:%M:%S")
    insert_query = """
        INSERT INTO record_tb
        (Date, Time, Start_Point, Finish_Point, Result_, Start_time, Finish_time, Duration) 
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
    """
    cursor.execute(insert_query, (dates, times, start_point, finish_point, result_stat, start_time, finish_time, duration))
    connection.commit()
    cursor.close()
    rospy.loginfo("Recorded in database - Start Point: {}, Finish Point: {}, Result: {}, Start Time: {}, Finish Time: {}, Duration: {}".format(start_point, finish_point, result_stat, start_time, finish_time, duration))
    if result_stat == "OK":
        start_point = finish_point
    else:
        start_point = "Home"

#----------------------------------------------------------------Main-------------------------------------------------------------#

def main():
    while not rospy.is_shutdown():
        read_mqtt()
        check_database()
        # rotate_direction()
        movebase_client()
        save_database()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        cursor.close()
        connection.close()
        rospy.loginfo("Navigation is Closed")
