#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def publish_cmd_vel():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_publisher', anonymous=True)

    # Create a publisher for the cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/velocity_controller/cmd_vel', Twist, queue_size=10)

    # Create Twist messages for forward, rotation, and stop
    forward_cmd = Twist()
    forward_cmd.linear.x = 0.2
    forward_cmd.linear.y = 0.0
    forward_cmd.linear.z = 0.0
    forward_cmd.angular.x = 0.0
    forward_cmd.angular.y = 0.0
    forward_cmd.angular.z = 0.0

    rotate_cmd = Twist()
    rotate_cmd.linear.x = 0.0
    rotate_cmd.linear.y = 0.0
    rotate_cmd.linear.z = 0.0
    rotate_cmd.angular.x = 0.0
    rotate_cmd.angular.y = 0.0
    rotate_cmd.angular.z = 1.0

    stop_cmd = Twist()

    # Move forward for 5 seconds
    start_time = time.time()
    while time.time() - start_time < 5.0:
        cmd_vel_pub.publish(forward_cmd)
        rospy.sleep(0.1)  # Sleep to control the publishing rate

    # Stop briefly before rotating
    cmd_vel_pub.publish(stop_cmd)
    rospy.sleep(1.0)  # Sleep for 1 second

    # Rotate for 5 seconds
    start_time = time.time()
    while time.time() - start_time < 5.0:
        cmd_vel_pub.publish(rotate_cmd)
        rospy.sleep(0.1)  # Sleep to control the publishing rate

    # Stop at the end
    cmd_vel_pub.publish(stop_cmd)

if __name__ == '__main__':
    try:
        publish_cmd_vel()
    except rospy.ROSInterruptException:
        pass