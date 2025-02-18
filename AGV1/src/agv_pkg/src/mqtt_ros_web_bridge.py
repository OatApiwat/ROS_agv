#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import String
import signal
import sys

def on_message(client, userdata, msg):
    payload = msg.payload.decode('utf-8')
    rospy.loginfo(f"Received Goal from MQTT: {payload}")

    # Check the message content and publish to ROS

    ros_publisher.publish(String(data= payload))
    rospy.loginfo("Published "+payload+ "to ROS topic")

def signal_handler(sig, frame):
    rospy.loginfo('Exiting the program...')
    client.loop_stop()  # Stop the MQTT client loop
    client.disconnect()  # Disconnect from the MQTT broker
    sys.exit(0)

def main():
    global client, ros_publisher

    # MQTT connection parameters
    host = 'localhost'
    port = 1883
    topic = '/mqtt_robot_target_topic'

    # Initialize the ROS node
    rospy.init_node('mqtt_ros_web_bridge', anonymous=True)
    ros_publisher = rospy.Publisher('/robot_target_topic', String, queue_size=10)

    # Create an MQTT client instance
    client = mqtt.Client()

    try:
        client.connect(host, port)
        rospy.loginfo("Successfully connected to MQTT broker")
    except ConnectionRefusedError:
        rospy.logwarn("Failed to connect to MQTT broker")
        sys.exit(1)

    # Set the MQTT callback function and subscribe to the topic
    client.on_message = on_message
    client.subscribe(topic)

    # Start the MQTT client loop
    client.loop_start()

    # Handle SIGINT signal for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    # Keep the ROS node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("mqtt_ros_bridge is Closed")
