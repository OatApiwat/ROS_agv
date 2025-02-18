#!/usr/bin/env python3
import rospy
from std_msgs.msg import String,Bool
import json
import time


ros_target_topic = '/ros_robot_target_topic'
ros_status_topic = '/ros_AGV_status'

# Global variables for ROS feedback
msg_feedback = None
msg_ir = None

# Callback for MQTT messages
def callback(data):
    # Publish working status to MQTT
    status_pub.publish("working")
    msg_target = json.loads(data.data)
    print(f"Received target: {msg_target}")
    
    # Publish target start to ROS topic
    target_pub.publish(msg_target['start'])
    print(f"Published start to ROS: {msg_target['start']}")
    
    # Wait for feedback and IR sensor messages
    wait_for_feedback_and_ir(msg_target['start'], msg_target['goal'])

# Wait for feedback and IR sensor messages
def wait_for_feedback_and_ir(start, goal):
    global msg_feedback, msg_ir
    print("waiting: msg_feedback")
    # Wait for feedback start
    msg_feedback = rospy.wait_for_message('/robot_feedback_box_topic', String).data
    print("msg_feedback: "+ msg_feedback)
    while True:
        msg_ir = rospy.wait_for_message('/ir_topic', Bool).data
        if msg_ir == True:
            break  # Exit the loop once msg_ir is true

    if msg_feedback == start and msg_ir:
        time.sleep(3)  # Wait for 3 seconds
        target_pub.publish(goal)  # Send goal to ROS
        print(f"Published goal to ROS: {goal}")
    
        # Wait for feedback goal
        msg_feedback = rospy.wait_for_message('/robot_feedback_box_topic', String).data
        while True:
            msg_ir = rospy.wait_for_message('/ir_topic', Bool).data
            if msg_ir == False:
                break  # Exit the loop once msg_ir is true

        if msg_feedback == goal and not msg_ir:
            time.sleep(3)  # Wait for 3 seconds
            target_pub.publish("HOME")  # Send "HOME" to ROS
            print("Published HOME to ROS")
        
            # Wait for feedback HOME
            msg_feedback = rospy.wait_for_message('/robot_feedback_box_topic', String).data

            if msg_feedback == "HOME":
                status_pub.publish("ready")
                print("Published ready status to MQTT")

def listener():
    # Subscribe to the topic
    rospy.Subscriber(ros_target_topic, String, callback)
    rospy.sleep(1)
    status_pub.publish("ready")
    print("Published ready status to MQTT")
    

# ROS setup
# Initialize the ROS node
rospy.init_node('json_listener', anonymous=True)
target_pub = rospy.Publisher('/robot_target_topic', String, queue_size=10)
status_pub = rospy.Publisher(ros_status_topic, String, queue_size=10)


if __name__ == "__main__":
    try:
        listener()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
