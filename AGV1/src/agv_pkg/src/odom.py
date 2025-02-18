#!/usr/bin/env python3
import rospy
from math import sin, cos, pi
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Float32MultiArray

odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
odomBroadcaster = TransformBroadcaster()

def cd_odom(msg):
    x = round(msg.data[0],5)
    y = round(msg.data[1],5)
    th = msg.data[2]*pi/180
    vel_x = round(msg.data[3],5)
    vel_y = round(msg.data[4],5)
    w_th = round(msg.data[5]*pi/180,5)
    now = rospy.Time.now()

    quaternion = Quaternion()
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = round(sin( th / 2 ),5)
    quaternion.w = round(cos( th / 2 ),5)
    odomBroadcaster.sendTransform(
        (x, y, 0),
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        now,
        'base_link',
        'odom'
        )
    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = 'odom'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0
    odom.pose.pose.orientation = quaternion
    odom.child_frame_id = 'base_link'
    odom.twist.twist.linear.x = vel_x
    odom.twist.twist.linear.y = vel_y
    odom.twist.twist.angular.z = w_th
    odomPub.publish(odom)


def main():
    rospy.init_node('diff_drive_odom_node',anonymous=True)
    while not rospy.is_shutdown():
        data = rospy.Subscriber('/forOdom_topic',Float32MultiArray,cd_odom)
        rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logdebug("Odom node error")

