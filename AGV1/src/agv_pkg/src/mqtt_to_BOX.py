#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json
import rospy  # สำหรับ ROS

# MQTT settings
mqtt_host = 'localhost'
mqtt_port = 1883
mqtt_target_topic = '/mqtt_robot_target_topic'
mqtt_status_topic = '/AGV_status'

# Callback สำหรับ MQTT messages
def on_mqtt_message(client, userdata, message):
    # แสดงข้อความที่ได้รับ
    rospy.loginfo(f"Received message: {message.payload.decode()}")
    
    # ส่งสถานะการทำงานไปยัง MQTT
    mqtt_client.publish(mqtt_status_topic, json.dumps({"msg_status": "working"}))

# MQTT setup
mqtt_client = mqtt.Client()
mqtt_client.on_message = on_mqtt_message
mqtt_client.connect(mqtt_host, mqtt_port)
mqtt_client.subscribe(mqtt_target_topic)
mqtt_client.loop_start()

if __name__ == "__main__":
    try:
        # Initialize ROS node
        rospy.init_node('mqtt_listener_node', anonymous=True)
        rospy.loginfo("MQTT listener node started.")

        # รัน ROS จนกว่าจะมีการหยุดโปรแกรม
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
