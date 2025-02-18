#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String

# MQTT settings
MQTT_HOST = 'localhost'
MQTT_PORT = 1883
MQTT_TOPIC_ROBOT_TARGET = '/mqtt_robot_target_topic'
MQTT_TOPIC_AGV_STATUS = '/mqtt_AGV_status'
MQTT_TOPIC_AGV_STAND = '/mqtt_robot_stand_topic'

# ROS topics
ROS_TOPIC_ROBOT_TARGET = '/ros_robot_target_topic'
ROS_TOPIC_AGV_STATUS = '/ros_AGV_status'
ROS_TOPIC_AGV_STAND = '/robot_feedback_box_topic'


# Initialize ROS node
rospy.init_node('mqtt_ros_bridge', anonymous=True)

# ROS Publishers and Subscribers
ros_robot_target_pub = rospy.Publisher(ROS_TOPIC_ROBOT_TARGET, String, queue_size=10)
ros_agv_status_sub = None
ros_agv_stand_sub = None

# MQTT Client
client = mqtt.Client()

# Callback when an MQTT message is received
def on_mqtt_message(client, userdata, msg):
    if msg.topic == MQTT_TOPIC_ROBOT_TARGET:
        # Forward MQTT message to ROS
        ros_msg = String(data=msg.payload.decode())
        ros_robot_target_pub.publish(ros_msg)
        rospy.loginfo(f"Forwarded MQTT message to ROS: {ros_msg.data}")

# Callback when a ROS message is received
def on_ros_agv_status_message(ros_msg):
    # Forward ROS message to MQTT
    mqtt_msg = ros_msg.data
    client.publish(MQTT_TOPIC_AGV_STATUS, mqtt_msg)
    rospy.loginfo(f"Forwarded ROS message to MQTT: {mqtt_msg}")

def on_ros_agv_stand_message(ros_msg):
    # Forward ROS message to MQTT
    mqtt_msg = ros_msg.data
    client.publish(MQTT_TOPIC_AGV_STAND, mqtt_msg)
    rospy.loginfo(f"Forwarded ROS message to MQTT: {mqtt_msg}")

# Setup MQTT
def setup_mqtt():
    client.on_message = on_mqtt_message
    client.connect(MQTT_HOST, MQTT_PORT, 60)
    client.subscribe(MQTT_TOPIC_ROBOT_TARGET)
    client.loop_start()

# Setup ROS Subscriber
def setup_ros():
    global ros_agv_status_sub,ros_agv_stand_sub
    ros_agv_status_sub = rospy.Subscriber(ROS_TOPIC_AGV_STATUS, String, on_ros_agv_status_message)
    ros_agv_stand_sub = rospy.Subscriber(ROS_TOPIC_AGV_STATUS, String, on_ros_agv_stand_message)

# Main function
def main():
    setup_mqtt()
    setup_ros()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()

if __name__ == '__main__':
    main()
