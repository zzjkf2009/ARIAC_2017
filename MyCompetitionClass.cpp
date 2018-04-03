/**
 * @Author: Zejiang Zeng <zzj>
 * @Date:   2018-04-03T14:21:49-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: MyCompetitionClass.cpp
 * @Last modified by:   zzj
 * @Last modified time: 2018-04-03T14:21:53-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */

 #include <algorithm>
 #include <vector>
 #include <cmath>
 #include <ros/ros.h>
 #include <osrf_gear/LogicalCameraImage.h>
 #include <osrf_gear/Order.h>
 #include <osrf_gear/Proximity.h>
 #include <sensor_msgs/JointState.h>
 #include <sensor_msgs/LaserScan.h>
 #include <sensor_msgs/Range.h>
 #include <std_msgs/Float32.h>
 #include <std_msgs/String.h>
 #include <std_srvs/Trigger.h>
 #include <trajectory_msgs/JointTrajectory.h>
 #include <osrf_gear/VacuumGripperControl.h>
 #include <osrf_gear/AGVControl.h>
 #include <osrf_gear/VacuumGripperState.h>
 #include <tf/transform_listener.h>
 #include <move_arm/Pick.h>
 #include <geometry_msgs/Pose.h>
 #include <vector>
 #include <string>
 #include <geometry_msgs/Point.h>
 #include <tf/transform_broadcaster.h>
 #include "MyCompetitionClass.h"

MyCompetitionClass::MyCompetitionClass(ros::NodeHandle & node)
        : current_score_(0)
{

        joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
                "/ariac/arm/command", 10);
        gripper_service = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
        tray_delivery_ = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
}

void MyCompetitionClass::current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
        if (msg->data != current_score_)
        {
                ROS_INFO_STREAM("Score: " << msg->data);
        }
        current_score_ = msg->data;
}

void MyCompetitionClass::competition_state_callback(const std_msgs::String::ConstPtr & msg) {
        if (msg->data == "done" && competition_state_ != "done")
        {
                ROS_INFO("Competition ended.");
        }
        competition_state_ = msg->data;
}

void MyCompetitionClass::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
        // ROS_INFO_STREAM("Received order:\n" << *order_msg);
        ROS_INFO_STREAM("order id is "<< order_msg->order_id);
        received_orders_.push_back(*order_msg);
        for(osrf_gear::Order order_ : received_orders_) {
                for(osrf_gear::Kit kit_ : order_.kits) {
                        for(osrf_gear::KitObject object_ : kit_.objects) {
                                order_pose_list.push_back(object_.pose);
                        }
                }
        }
}

void MyCompetitionClass::orderPoseBroadcaster(geometry_msgs::Pose& msg, const int& x) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg.position.x, msg.position.y, msg.position.z));;
        tf::Quaternion q;
        quaternionMsgToTF(msg.orientation, q);
        transform.setRotation(q);
        std::string name = "/order";
        std::string num = std::to_string(x);
        std::string order_name = name + num;
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/agv1_load_point_frame", order_name));
}

geometry_msgs::Pose MyCompetitionClass::desired_part_pose(const tf::TransformListener &listener,const int &x) {

        std::string name = "/order";
        std::string num = std::to_string(x);
        std::string order_name = name + num;
        geometry_msgs::Pose pose;

        //tf::TransformListener listener;       // initialize tf listener
        tf::StampedTransform transform_order;  // initialize transform between gripper and order part
        try {
                //listener.waitForTransform("/order0","/world",ros::Time(0), ros::Duration(10.0));
                listener.lookupTransform(order_name,"/world",ros::Time(0),transform_order);
                pose.position.x = transform_order.getOrigin().x();
                pose.position.y = transform_order.getOrigin().y() - 0.16;
                pose.position.z = -transform_order.getOrigin().z();
                pose.orientation.x = transform_order.getRotation().getX();
                pose.orientation.y = transform_order.getRotation().getY();
                pose.orientation.z = transform_order.getRotation().getZ();
                pose.orientation.w = transform_order.getRotation().getW();
                return pose;
        }
        catch(tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                //ros::Duration(1.0).sleep();
        }
}

void MyCompetitionClass::joint_state_callback(
        const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
        // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
        current_joint_states_ = *joint_state_msg;
}

void MyCompetitionClass::logical_camera_callback(
        const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
{
        ROS_INFO_STREAM("Logical camera: '" << image_msg->models.size() << "' objects.");
}

void MyCompetitionClass::break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
        if (msg->object_detected) { // If there is an object in proximity.
                ROS_INFO("Break beam triggered.");
        }
}

void MyCompetitionClass::proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
        if ((msg->max_range - msg->range) > 0.01) { // If there is an object in proximity.
                ROS_INFO_("Proximity sensor sees something.");
        }
}

void MyCompetitionClass::laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
        size_t number_of_valid_ranges = std::count_if(
                msg->ranges.begin(), msg->ranges.end(), [] (int i){return std::isfinite(i); });
        if (number_of_valid_ranges > 0) {
                ROS_INFO("Laser profiler sees something.");
        }
}
