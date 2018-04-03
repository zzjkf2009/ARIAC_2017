/**
 * @Author: Zejiang Zeng <zzj>
 * @Date:   2018-04-03T13:34:56-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: MyCompetitionClass.h
 * @Last modified by:   zzj
 * @Last modified time: 2018-04-03T13:35:05-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */

#ifndef   MYCOMPETITIONCLASS_H
#define  MYCOMPETITIONCLASS_H


#include <vector>
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
#include <string>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

class MyCompetitionClass
{
public:
/**
 * [MyCompetitionClass Constructor to initialize the publisher or serivce]
 * @param  node [ros::NodeHandle]
 */
explicit MyCompetitionClass(ros::NodeHandle & node);
/**
 * [current_score_callback callback function that read the current sorce]
 * @param msg [std_msgs::Float32]
 */
void current_score_callback(const std_msgs::Float32::ConstPtr & msg);
/**
 * [competition_state_callback callback function to read the state of the competition]
 * @param msg [std_msgs::String]
 */
void competition_state_callback(const std_msgs::String::ConstPtr & msg);
/**
 * [order_callback Read the order of the competition, and store the pose into a container]
 * @param order_msg [osrf_gear::Order]
 */
void order_callback(const osrf_gear::Order::ConstPtr & order_msg);
/**
 * [orderPoseBroadcaster Broadcaster tf from the pose got from the order pose list]
 * @param msg [geometry_msgs::Pose]
 * @param x   [part numnber]
 */
void orderPoseBroadcaster(geometry_msgs::Pose& msg, const int& x);
/**
 * [desired_part_pose Publish pose info by listen the transform between the
 * target frame to the source frame (/world)]
 * @param  listener [tf::TransformListener]
 * @param  x        [index number]
 * @return          [geometry_msgs::Pose]
 */
geometry_msgs::Pose desired_part_pose(const tf::TransformListener &listener,const int &x);
/**
 * [joint_state_callback reveive the joint state]
 * @param joint_state_msg [sensor_msgs::JointState]
 */
void joint_state_callback(
        const sensor_msgs::JointState::ConstPtr & joint_state_msg);
/**
 * [logical_camera_callback reveive the info frm the logical_camera]
 * @param image_msg [osrf_gear::LogicalCameraImage]
 */
void logical_camera_callback(
        const osrf_gear::LogicalCameraImage::ConstPtr & image_msg);
/**
 * [break_beam_callback Called when a new Proximity message is received.]
 * @param msg [osrf_gear::Proximity]
 */
void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg);
/**
 * [proximity_sensor_callback reveive proximity sensor info ]
 * @param msg [sensor_msgs::Range]
 */
void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg);
/**
 * [laser_profiler_callback reveive proximity sensor info]
 * @param msg [sensor_msgs::LaserScan]
 */
void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg);
std::vector<geometry_msgs::Pose> order_pose_list;
private:
std::string competition_state_;
double current_score_;
ros::Publisher joint_trajectory_publisher_;
std::vector<osrf_gear::Order> received_orders_;
sensor_msgs::JointState current_joint_states_;
};
#endif
