#include "ros/ros.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Mavlink.h>
#include <std_msgs/Bool.h>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argv,char** argc)
{
    ros::init(argv, argc, "arm");
    ros::NodeHandle nh;
    int UAV_ID;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    ros::Publisher takeoff_complete_pub = nh.advertise<std_msgs::Bool>("takeoff_complete", 10);  // Publisher for takeoff complete

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    ros::Rate rate(100.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ros::param::get("UAV_ID", UAV_ID);
    ROS_INFO("Wait for setting origin and home position...");
    std::string mavlink_topic = std::string("/MAV") + std::to_string(UAV_ID) + std::string("/mavlink/to");
    ros::topic::waitForMessage<mavros_msgs::Mavlink>(mavlink_topic, ros::Duration(10.0));
    ROS_INFO("Message received or timeout reached. Continuing execution.");
    sleep(2);

    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("GUIDED enabled");
    }

    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    }

    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 0.5;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;

    if (takeoff_cl.call(srv_takeoff) && srv_takeoff.response.success) {
        ROS_INFO("takeoff_service success!  %d", srv_takeoff.response.success);

        // Publish the takeoff complete signal
        std_msgs::Bool takeoff_complete_msg;
        takeoff_complete_msg.data = true;
        takeoff_complete_pub.publish(takeoff_complete_msg);
    } else {
        ROS_ERROR("Takeoff failed !!! ");
    }

    ROS_INFO("hovering");
    sleep(30);
    ros::Time time_out = ros::Time::now();

    return 0;
}

