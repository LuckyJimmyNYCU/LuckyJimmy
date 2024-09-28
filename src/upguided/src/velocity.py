#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped

class VelocityCalculator:
    def __init__(self):
        rospy.init_node('velocity_calculator', anonymous=True)

        # Subscribers for TUR1, MAV1 and MAV2 poses
        self.pose_sub_tur1 = rospy.Subscriber('/vrpn_client_node/TUR1/pose', PoseStamped, self.pose_callback_tur1)
        self.pose_sub_mav1 = rospy.Subscriber('/vrpn_client_node/MAV1/pose', PoseStamped, self.pose_callback_mav1)
        self.pose_sub_mav2 = rospy.Subscriber('/vrpn_client_node/MAV2/pose', PoseStamped, self.pose_callback_mav2)

        # Publishers for both TUR1 and MAV1 velocities
        self.velocity_pub_tur1 = rospy.Publisher('/turtlebot/vel_data_tur1', TwistStamped, queue_size=10)
        self.velocity_pub_mav1 = rospy.Publisher('/h420/vel_data', TwistStamped, queue_size=10)
        self.velocity_pub_mav2 = rospy.Publisher('/h250/vel_data', TwistStamped, queue_size=10)

        self.prev_pose_tur1 = None
        self.prev_time_tur1 = None

        self.prev_pose_mav1 = None
        self.prev_time_mav1 = None

        self.prev_pose_mav2 = None
        self.prev_time_mav2 = None

        self.rate = rospy.Rate(200)  # Set the loop rate to 200 Hz

    def pose_callback_tur1(self, pose_msg):
        self.calculate_velocity(pose_msg, 'TUR1')

    def pose_callback_mav1(self, pose_msg):
        self.calculate_velocity(pose_msg, 'MAV1')

    def pose_callback_mav2(self, pose_msg):
        self.calculate_velocity(pose_msg, 'MAV2')

    def calculate_velocity(self, pose_msg, source):
        if source == 'TUR1':
            prev_pose = self.prev_pose_tur1
            prev_time = self.prev_time_tur1
            velocity_pub = self.velocity_pub_tur1

        elif source == 'MAV1':
            prev_pose = self.prev_pose_mav1
            prev_time = self.prev_time_mav1
            velocity_pub = self.velocity_pub_mav1

        elif source == 'MAV2':
            prev_pose = self.prev_pose_mav2
            prev_time = self.prev_time_mav2
            velocity_pub = self.velocity_pub_mav2

        if prev_pose is not None and prev_time is not None:
            dt = (pose_msg.header.stamp - prev_time).to_sec()  # Calculate time difference
            dx = pose_msg.pose.position.x - prev_pose.pose.position.x
            dy = pose_msg.pose.position.y - prev_pose.pose.position.y
            dz = pose_msg.pose.position.z - prev_pose.pose.position.z
            velocity_x = dx / dt
            velocity_y = dy / dt
            velocity_z = dz / dt

            # Publish velocity as TwistStamped message
            velocity_msg = TwistStamped()
            velocity_msg.header.stamp = rospy.Time.now()
            velocity_msg.twist.linear.x = velocity_x
            velocity_msg.twist.linear.y = velocity_y
            velocity_msg.twist.linear.z = velocity_z
            velocity_pub.publish(velocity_msg)

        if source == 'TUR1':
            self.prev_pose_tur1 = pose_msg
            self.prev_time_tur1 = pose_msg.header.stamp
        elif source == 'MAV1':
            self.prev_pose_mav1 = pose_msg
            self.prev_time_mav1 = pose_msg.header.stamp
        elif source == 'MAV2':
            self.prev_pose_mav2 = pose_msg
            self.prev_time_mav2 = pose_msg.header.stamp
        self.rate.sleep()  # Sleep to maintain the specified loop rate

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        velocity_calculator = VelocityCalculator()
        velocity_calculator.spin()
    except rospy.ROSInterruptException:
    	pass

