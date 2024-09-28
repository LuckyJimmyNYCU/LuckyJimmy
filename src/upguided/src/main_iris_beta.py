#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import TwistStamped, Twist, PointStamped, PoseStamped
from tf.transformations import euler_from_quaternion
import math

# define variables
az = 0
dvx, dvy, dvz, hx, hy, hz, hvx, hvy, x, y, z = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
ux = uy = uz = 0
yaw = 0
hbvx, hbvz = 0, 0
hvxf, hvzf = 0, 0
vx, vy, vz = 0, 0, 0
distance = 0
takeoff_complete = False
pos_gain = 1.0  # You can adjust this gain to tune control behavior
vel_gain = 0.5
# Initialize publisher
twist_pub = rospy.Publisher('/MAV2/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
husky_pub = rospy.Publisher('/observer1/cmd_vel', Twist, queue_size=10)

twist = TwistStamped()
husky = Twist()

# Callback to receive path controller velocities
def husky_vel_callback(msg):
    global hvxf, hvzf
    hvxf = msg.data[0]  # Linear velocity from path controller
    hvzf = msg.data[1]  # Angular velocity from path controller

# Callback to handle keyboard input
def input_callback(msg):
    global dvx, dvy, dvz, az, hbvx, hbvz
    linear = msg.data
    dvx, dvy, dvz, az, hbvx, hbvz = linear[0], linear[1], linear[2], linear[3], linear[4], linear[5]
    return dvx, dvy, dvz, az, hbvx, hbvz

def h250_vel_callback(data):
    global vx, vy, vz    
    vx = data.twist.linear.x
    vy = data.twist.linear.y
    vz = data.twist.linear.z

def turtlebot_vel_callback(data):
    global hvx, hvy    
    hvx = data.twist.linear.x
    hvy = data.twist.linear.y

def turtlebot_pos_callback(data):
    global hx, hy, hz    
    hx = data.pose.position.x
    hy = data.pose.position.y
    hz = data.pose.position.z

def iris_pos_callback(data):
    global x, y, z, roll, pitch, yaw
    x = data.pose.position.x
    y = data.pose.position.y
    z = data.pose.position.z
    orientation = data.pose.orientation
    roll, pitch, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

# Callback to update takeoff status
def takeoff_complete_callback(msg):
    global takeoff_complete
    takeoff_complete = msg.data

# Calculate control command
def calculate():
    global ux, uy, uz, takeoff_complete, distance

    if not takeoff_complete:
        # Pre-takeoff: UAV should remain stationary or perform necessary actions
        hvxf = 0
        hvzf = 0

    else:
        # Post-takeoff: Hover 0.75m above the Husky's current position    
        ux = pos_gain * (hx - x)
        uy = pos_gain * (hy - y)
        uz = pos_gain * (hz + 0.75 - z)

	ux += vel_gain * hvx
	uy += vel_gain * hvy

    # Publish UAV control command
    twist.twist.linear.x = ux
    twist.twist.linear.y = uy
    twist.twist.linear.z = uz
    twist.twist.angular.z = az

    # Publish turtlebot control command
    husky.linear.x = hvxf
    husky.angular.z = hvzf
# Initialize node and subscribers
rospy.init_node("get_all_data")

# Subscribe to topics
rospy.Subscriber("/vrpn_client_node/MAV2/pose", PoseStamped, iris_pos_callback)
rospy.Subscriber("/vrpn_client_node/TUR1/pose", PoseStamped, turtlebot_pos_callback)
rospy.Subscriber("/h250/vel_data", TwistStamped, h250_vel_callback)
rospy.Subscriber('/keyboard_input', Float32MultiArray, input_callback)
rospy.Subscriber("/turtlebot/vel_data", TwistStamped, turtlebot_vel_callback)
rospy.Subscriber("/takeoff_complete", Bool, takeoff_complete_callback)

# Main loop
rate = rospy.Rate(80)
while not rospy.is_shutdown():
    calculate()
    print("ux = {:.2f}, uy = {:.2f}, uz = {:.2f}, distance = {:.2f}".format(ux, uy, uz, distance))
    
    twist_pub.publish(twist)
    husky_pub.publish(husky)
    
    rate.sleep()

