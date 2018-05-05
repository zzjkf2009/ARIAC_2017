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
 #include "tf2_msgs/TFMessage.h"

MyCompetitionClass::MyCompetitionClass(ros::NodeHandle & node)
        : current_score_(0)
{

        joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
                "/ariac/arm/command", 10);

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
        int tem_priority_num = 0;
        std::string order_name ("Nothing");
        order Order;
        if(order_name.compare(order_msg->order_id) != 0) {
                osrf_gear::Order order_ = *order_msg;
                for(osrf_gear::Kit kit_ : order_.kits) {
                        Kit kit;
                        for(osrf_gear::KitObject object_ : kit_.objects) {
                                Order.pose = object_.pose;
                                Order.type = object_.type;
                                kit.order_in_kit_list.push_back(Order);
                                kit.priority = priority_num;
                        }
                        kit_priority_queue.insert(std::pair<int,Kit>(kit.priority,kit));
                        priority_num++;
                }
                order_name = order_msg->order_id;
        }
}

void MyCompetitionClass::orderPoseBroadcaster(geometry_msgs::Pose& msg, const int& x,const int& tray_num) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg.position.x, msg.position.y + 0.16, msg.position.z));
        tf::Quaternion q;
        quaternionMsgToTF(msg.orientation, q);
        transform.setRotation(q);
        //transform.setRotation(tf::Quaternion(0, 0, 0, 0));
        std::string name = "/order";
        std::string num = std::to_string(x);
        std::string order_name = name + num;
        if (tray_num == 1) {
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/agv1_load_point_frame", order_name));
                ROS_INFO_STREAM("assigned to AGV 1");
        }

        else {
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/agv2_load_point_frame", order_name));
                ROS_INFO_STREAM("assigned to AGV 2");
        }
        ros::Duration(0.2).sleep();
}


geometry_msgs::Pose MyCompetitionClass::desired_part_pose(const tf::TransformListener &listener,const int &x,const int& tray_num) {

        std::string name = "/order";
        std::string num = std::to_string(x);
        std::string order_name = name + num;
        geometry_msgs::Pose pose;

        //tf::TransformListener listener;       // initialize tf listener
        tf::StampedTransform transform_order;  // initialize transform between gripper and order part
        try {
                //listener.waitForTransform(order_name,"/world",ros::Time(0), ros::Duration(0.1));

                if (tray_num == 1) {
                        listener.lookupTransform("/world",order_name,ros::Time(0),transform_order);
                        pose.position.x = transform_order.getOrigin().x();
                        pose.position.y = transform_order.getOrigin().y();
                        /**
                              listener.lookupTransform(order_name,"/world",ros::Time(0),transform_order);
                              if( abs(transform_order.getOrigin().x()) < 1.0 && abs(transform_order.getOrigin().y()) > 2.5) {
                                      pose.position.x = transform_order.getOrigin().x();
                                      pose.position.y = transform_order.getOrigin().y();
                              }
                              else {
                                      listener.lookupTransform("/world",order_name,ros::Time(0),transform_order);
                                      pose.position.x = transform_order.getOrigin().x();
                                      pose.position.y = transform_order.getOrigin().y();
                              }**/

                }
                else {
                        listener.lookupTransform("/world",order_name,ros::Time(0),transform_order);
                        pose.position.x = transform_order.getOrigin().x();
                        pose.position.y = transform_order.getOrigin().y();
                }

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
        //  ROS_INFO_STREAM("Logical camera: '" << image_msg->models.size() << "' objects.");
}

void MyCompetitionClass::break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
        if (msg->object_detected) { // If there is an object in proximity.
                ROS_INFO("Break beam triggered.");
        }
}

void MyCompetitionClass::proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
        if ((msg->max_range - msg->range) > 0.01) { // If there is an object in proximity.
                ROS_INFO_STREAM("Proximity sensor sees something.");
        }
}

void MyCompetitionClass::laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
        size_t number_of_valid_ranges = std::count_if(
                msg->ranges.begin(), msg->ranges.end(), [] (int i){return std::isfinite(i); });
        if (number_of_valid_ranges > 0) {
                ROS_INFO("Laser profiler sees something.");
        }
}

void MyCompetitionClass::agv1_state_callback(const std_msgs::String::ConstPtr &msg){
        std::string state("ready_to_deliver");
        if(state.compare(msg->data) == 0)
                agv1_ready_position = true;
        else
                agv1_ready_position = false;
}

void MyCompetitionClass::agv2_state_callback(const std_msgs::String::ConstPtr &msg){
        std::string state("ready_to_deliver");
        if(state.compare(msg->data) == 0)
                agv2_ready_position = true;
        else
                agv2_ready_position = false;
}

// callback function part_detect filters information about parts
// from other transfromation in /tf
void MyCompetitionClass:: agv_faulty_part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {
        tf::TransformListener listener;
        //ROS_INFO_STREAM("Let's see the faulty part");
        for (auto possible_part : msg->transforms) {
                if (possible_part.header.frame_id == "quality_control_sensor_1_frame" || possible_part.header.frame_id == "quality_control_sensor_2_frame") {
                        faulty_on_agv = true;
                        ROS_INFO_STREAM("faulty part is found");
                        tf::StampedTransform temp_transform;
                        geometry_msgs::TransformStamped temp_part;
                        listener.waitForTransform("/world",possible_part.child_frame_id,ros::Time(0), ros::Duration(0.1));
                        listener.lookupTransform("/world", possible_part.child_frame_id, ros::Time(0), temp_transform);
                        tf::transformStampedTFToMsg(temp_transform, temp_part);
                        faulty_pose.position.x = temp_part.transform.translation.x;
                        faulty_pose.position.y = temp_part.transform.translation.y;
                        faulty_pose.position.z = 0.745;
                        break;
                }
        }

}
