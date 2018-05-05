/**
 * @Author: Zejiang Zeng <zzj>
 * @Date:   2018-04-03T12:10:57-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: qual2a.cpp
 * @Last modified by:   zzj
 * @Last modified time: 2018-04-03T12:16:12-04:00
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
#include <queue>
/**
 * [start_competition Start the competition by waiting for and then calling the start ROS Service.]
 * @param node [roshandle node]
 */
void start_competition(ros::NodeHandle & node) {
        // Create a Service client for the correct service, i.e. '/ariac/start_competition'.
        ros::ServiceClient start_client =
                node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
        // If it's not already ready, wait for it to be ready.
        // Calling the Service using the client before the server is ready would fail.
        if (!start_client.exists()) {
                ROS_INFO("Waiting for the competition to be ready...");
                start_client.waitForExistence();
                ROS_INFO("Competition is now ready.");
        }
        ROS_INFO("Requesting competition start...");
        std_srvs::Trigger srv; // Combination of the "request" and the "response".
        start_client.call(srv); // Call the start Service.
        if (!srv.response.success) { // If not successful, print out why.
                ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
        } else {
                ROS_INFO("Competition started!");
        }
}
template <class T>
struct Greater {
        bool operator()(const T& p1,const T& p2){
                return (p1.priority > p2.priority);
        }
};
class Order {
public:
Order(geometry_msgs::Pose p, int i) : pose(p),priority(i){
}
geometry_msgs::Pose pose;
int priority;
/*
   bool operator == (const order &s2) const {
        return (priority == s2.priority);
   }
   bool operator > (const order &s2) const {
        return (priority > s2.priority);
   }
   bool operator < (const order &s2) const {
        return (priority < s2.priority);
   }*/
};
/**
 * [MyCompetitionClass  Class that can hold state and provide methods that handle incoming data.]
 */
class MyCompetitionClass
{
public:
std::priority_queue <Order,std::vector<Order>,Greater<Order> > order_priority_queue;
explicit MyCompetitionClass(ros::NodeHandle & node)
        : current_score_(0), has_been_zeroed_(false)
{

        joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
                "/ariac/arm/command", 10);
        gripper_service = node.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
        tray_delivery_ = node.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
}

void current_score_callback(const std_msgs::Float32::ConstPtr & msg) {
        if (msg->data != current_score_)
        {
                ROS_INFO_STREAM("Score: " << msg->data);
        }
        current_score_ = msg->data;
}

void competition_state_callback(const std_msgs::String::ConstPtr & msg) {
        if (msg->data == "done" && competition_state_ != "done")
        {
                ROS_INFO("Competition ended.");
        }
        competition_state_ = msg->data;
}

// Create a pair of pose to record the pose info for where need to pick/place part
// first is the pick , second is place
std::pair<geometry_msgs::Pose,geometry_msgs::Pose> pose_pair_info;
std::vector<std::pair<geometry_msgs::Pose,geometry_msgs::Pose> > pick_place_list;
std::vector<geometry_msgs::Pose> order_pose_list;

//geometry_msgs::Pose desired_part_pose(const geometry_msgs::Pose &msg,const int &x) {
geometry_msgs::Pose desired_part_pose(const tf::TransformListener &listener,const int &x) {

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

geometry_msgs::Pose assign_pose(const float& x, const float& y,const float& z) {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        return pose;
}


std::vector<std::vector<float> > bin = {{-0.2,0.33,0.745},{-0.3,0.13,0.724},{-0.1,-0.335,0.745},{-0.233,-0.468,0.745},{-0.36,-0.335,0.745}};
/// Called when a new Order message is received.
void order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
        // ROS_INFO_STREAM("Received order:\n" << *order_msg);
        ROS_INFO_STREAM("order id is "<< order_msg->order_id);
        bool record_orders = false;
        int priority_num = 0;

        received_orders_.push_back(*order_msg);
        for(osrf_gear::Order order_ : received_orders_) {
                priority_num++;
                for(osrf_gear::Kit kit_ : order_.kits) {
                        for(osrf_gear::KitObject object_ : kit_.objects) {
                                order_pose_list.push_back(object_.pose);
                                Order order(object_.pose,priority_num);
                                order_priority_queue.push(order);
                        }
                }
        }

        // ROS_INFO_STREAM("There are " << object_list.size()<<" objects\n" );
//ROS_INFO_STREAM("There are " << pick_place_list.size()<<" objects\n" );
//ROS_INFO_STREAM("First x position is  " << pick_place_list[1].second.position.x);
//ROS_INFO_STREAM("First x position is  " << pick_place_list[1].second.position.y);
//ROS_INFO_STREAM("First x position is  " << pick_place_list[1].second.position.z);
}

void orderPoseBroadcaster(geometry_msgs::Pose& msg, const int& x) {
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


move_arm::Pick action(ros::NodeHandle & node,geometry_msgs::Pose& msg){
        //  ROS_INFO_STREAM("size of this shit list in action "<< pick_place_list.size());
        //  ros::ServiceClient client = node.serviceClient<move_arm::Pick>("/move_arm/toPose");
        move_arm::Pick Movesrv;
        Movesrv.request.pose.position.x = 0.2; //msg.position.x;
        Movesrv.request.pose.position.y = 3.5; //msg.position.y;
        Movesrv.request.pose.position.z = 0.75; //msg.position.z;
        Movesrv.request.mode = 2;
        return Movesrv;
        //client.call(Movesrv);
}

/**
 * [joint_state_callback Callback funtion, called when a new JointState message is received.
 * Used as joint controller for each robot]
 * @param joint_state_msg [sensor_msgs::JointState]
 */
void joint_state_callback(
        const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{
        ROS_INFO_STREAM_THROTTLE(10,
                                 "Joint States (throttled to 0.1 Hz):\n" << *joint_state_msg);
        // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
        current_joint_states_ = *joint_state_msg;
        if (!has_been_zeroed_) {
                has_been_zeroed_ = true;
                ROS_INFO("Sending arm to zero joint positions...");
                send_arm_to_zero_state();
        } /*
             else {
                // Create a message to send.
                trajectory_msgs::JointTrajectory traj;
                traj.joint_names.clear();
                traj.joint_names.push_back("elbow_joint");
                traj.joint_names.push_back("linear_arm_actuator_joint");
                traj.joint_names.push_back("shoulder_lift_joint");
                traj.joint_names.push_back("shoulder_pan_joint");
                traj.joint_names.push_back("wrist_1_joint");
                traj.joint_names.push_back("wrist_2_joint");
                traj.joint_names.push_back("wrist_3_joint");
                place_kit_tray1();
                //std::vector<double> goal = {1.76, 2.06,-0.63, 1.5, 3.27, -1.51, 0.0};
                std::vector<double> goal = {1.60, 2.1,-0.70, 1.4, 3.27, -1.51, 0.0};
                if(num_tools_in_tray_ == 0 && order_0_ == false) {
                        grasp_bin7();
                        goal[3] = 1.37;
                        std::vector<double> middle_p = {1.5, 0.42, -1.0, 1.5, 3.58,-1.51,0.0};
                        std::vector<double> p1 = {1.76, 0.42, -0.47, 3.23,3.58,-1.51,0.0};
                        if(gripper_state_attatch_ == false) {
                                move_to(p1,traj);
                        }
                        else
                                move_to(middle_p,goal,traj,joint_state_msg,order_0_,pass_order0[1]);
                }
                if(num_tools_in_tray_ == 1 && order_1_ == false) {
                        grasp_bin7();
                        goal[3] = 1.40;
                        std::vector<double> middle_p = {1.76, 0.1, -1.0, 1.5, 3.58,-1.51,0.0};
                        std::vector<double> p2 = {2.0, 0.1, -0.5, 3.0,3.58,-1.51,0.0};
                        if(gripper_state_attatch_ == false) {

                                move_to(middle_p,p2,traj,joint_state_msg,pass_order1[0]);
                        }
                        else
                                move_to(middle_p,goal,traj,joint_state_msg,order_1_,pass_order1[1]);
                }
                if(num_tools_in_tray_ == 2 && order_2_ == false) {
                        grasp_bin6();
                        goal[3] = 1.43;
                        std::vector<double> middle_p = {1.35, -0.46, -1.0, 2.0, 3.58,-1.51,0.0};
                        std::vector<double> p2 = {2.0, -0.37, -0.50, 3.50,3.52,-1.51,0.0};
                        if(gripper_state_attatch_ == false) {
                                move_to(middle_p,p2,traj,joint_state_msg,pass_order2[0]);
                        }
                        else
                                move_to(middle_p,goal,traj,joint_state_msg,order_2_,pass_order2[1]);
                }
                if(num_tools_in_tray_ == 3 && order_3_ == false) {
                        grasp_bin6();
                        goal[3] = 1.47;
                        std::vector<double> middle_p = {1.20, -0.5, -1.1, 2.0, 3.58,-1.51,0.0};
                        std::vector<double> p2 = {2.13, -0.35, -0.5, 3.02,3.27,-1.51,0.0};
                        if(gripper_state_attatch_ == false) {
                                move_to(middle_p,p2,traj,joint_state_msg,pass_order3[0]);
                        }
                        else
                                move_to(middle_p,goal,traj,joint_state_msg,order_3_,pass_order3[1]);
                }
                if(num_tools_in_tray_ == 4 && order_4_ == false) {
                        grasp_bin6();
                        goal[3] = 1.50;
                        std::vector<double> middle_p = {1.35, -0.30, -1.0, 2.0, 3.58,-1.51,0.0};
                        std::vector<double> p2 = {2.26, -0.38, -0.50, 3.00,3.14,-1.51,0.0};
                        if(gripper_state_attatch_ == false) {
                                move_to(middle_p,p2,traj,joint_state_msg,pass_order4[0]);
                        }
                        else
                                move_to(middle_p,goal,traj,joint_state_msg,order_4_,pass_order4[1]);
                }
             } */
}



/// Create a JointTrajectory with all positions set to zero, and command the arm.
void send_arm_to_zero_state() {
        // Create a message to send.
        trajectory_msgs::JointTrajectory msg;

        // Fill the names of the joints to be controlled.
        // Note that the vacuum_gripper_joint is not controllable.
        msg.joint_names.clear();
        msg.joint_names.push_back("elbow_joint");
        msg.joint_names.push_back("linear_arm_actuator_joint");
        msg.joint_names.push_back("shoulder_lift_joint");
        msg.joint_names.push_back("shoulder_pan_joint");
        msg.joint_names.push_back("wrist_1_joint");
        msg.joint_names.push_back("wrist_2_joint");
        msg.joint_names.push_back("wrist_3_joint");
        // Create one point in the trajectory.
        msg.points.resize(1);
        // Resize the vector to the same length as the joint names.
        // Values are initialized to 0.
        //msg.points[0].positions = {1.76, 0.48, -0.47, 3.23,3.58,-1.51,0.0};
        msg.points[0].positions = {1.56, 0.48, -1.13, 3.14,3.58,-1.51,0.0};
        // How long to take getting to the point (floating point seconds).
        msg.points[0].time_from_start = ros::Duration(duration_time_);
        msg.header.stamp = ros::Time::now() + ros::Duration();
        ROS_INFO_STREAM("Sending command:\n" << msg);
        joint_trajectory_publisher_.publish(msg);
}


/// Called when a new LogicalCameraImage message is received.
void logical_camera_callback(
        const osrf_gear::LogicalCameraImage::ConstPtr & image_msg)
{
        ROS_INFO_STREAM("Logical camera: '" << image_msg->models.size() << "' objects.");
}

/// Called when a new
void tray_logical_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
        //ROS_INFO_STREAM("Logical camera_2: '" << image_msg->models.size() << "' objects.");
        num_tools_in_tray_ = image_msg->models.size() -2;
        if(num_tools_in_tray_ == 5) {
                osrf_gear::AGVControl srv;
                srv.request.kit_type = "order_0_kit_0";
                tray_delivery_.call(srv);
        }
}

// Let the arm move along a two points trjectory with setting the order
void move_to(const std::vector<double> &v1,const std::vector<double> &v2, trajectory_msgs::JointTrajectory& traj,
             const sensor_msgs::JointState::ConstPtr& joint_state_msg,bool &order,int &pass) {
        traj.points.clear();
        traj.points.resize(1);
        if(pass == 0) {
                ROS_INFO("Move to middle point");
                pass = isclose(v1,joint_state_msg->position);
                traj.points[0].positions = v1;
                traj.points[0].time_from_start = ros::Duration(duration_time_/2);
                joint_trajectory_publisher_.publish(traj);
        } else {
                ROS_INFO("Move to goal point");
                traj.points[0].positions = v2;
                traj.points[0].time_from_start = ros::Duration(duration_time_/2);
                joint_trajectory_publisher_.publish(traj);
                order = true;
        }
        //traj.header.stamp = ros::Time::now() + ros::Duration();
}

// Let the arm move along a two points trjectory without setting the order
void move_to(const std::vector<double> &v1,const std::vector<double> &v2, trajectory_msgs::JointTrajectory& traj,
             const sensor_msgs::JointState::ConstPtr& joint_state_msg,int &pass) {
        traj.points.clear();
        traj.points.resize(1);
        if(pass == 0) {
                ROS_INFO("Move to middle point");
                pass = isclose(v1,joint_state_msg->position);
                traj.points[0].positions = v1;
                traj.points[0].time_from_start = ros::Duration(duration_time_/2);
                joint_trajectory_publisher_.publish(traj);
        } else {
                ROS_INFO("Move to goal point");
                traj.points[0].positions = v2;
                traj.points[0].time_from_start = ros::Duration(duration_time_);
                joint_trajectory_publisher_.publish(traj);
        }
        //traj.header.stamp = ros::Time::now() + ros::Duration();
}


// Let the arm move to one point
void move_to(const std::vector<double> &v1, trajectory_msgs::JointTrajectory& traj) {
        traj.points.clear();
        traj.points.resize(1);
        traj.points[0].positions = v1;
        traj.points[0].time_from_start = ros::Duration(duration_time_/2);
        //traj.header.stamp = ros::Time::now() + ros::Duration();
        joint_trajectory_publisher_.publish(traj);
}

int isclose(const std::vector<double> &v1,const std::vector<double> &v2) {
        for(int i = 0; i< v1.size(); i++) {
                if(abs(v1[i] - v2[i]) < 0.05)
                        continue;
                else
                        return 0;
        }
        return 1;
}




/// Called when a new Proximity message is received.
void break_beam_callback(const osrf_gear::Proximity::ConstPtr & msg) {
        if (msg->object_detected) { // If there is an object in proximity.
                ROS_INFO("Break beam triggered.");
        }
}

void gripper_state_attatch_callback(const osrf_gear::VacuumGripperState::ConstPtr & msg) {
        gripper_state_attatch_ = msg->attached;
}
/*
 * @brief This function is to enable the vacuum gripper to grasp kit
 */
void grasp_kit() {

        // set up parameter of gripper service
        osrf_gear::VacuumGripperControl srv;
        srv.request.enable = true;

        gripper_service.call(srv);
}

/*
 * @brief This function is to disable the vacuum gripper to release the kit
 */
void release_kit() {
        // set up parameter of gripper service to disable vacuum gripper
        osrf_gear::VacuumGripperControl srv;
        srv.request.enable =false;
        gripper_service.call(srv);
}
/*
 * @brief Check whether relative position under tolerance with difference upper and lower bound
 * @param upper_bound: the upper_bound of tolerance
 * @param lower_bound: the lower bound of tolerance
 * @param relative_pose: relative position in x, y, z direction
 * @return bool
 */
bool under_tolerance(const geometry_msgs::Point& upper_bound, \
                     const geometry_msgs::Point& lower_bound, const geometry_msgs::Point& relative_pose) {
        if (relative_pose.x < upper_bound.x && relative_pose.x > lower_bound.x \
            && relative_pose.y < upper_bound.y && relative_pose.y > lower_bound.y \
            && relative_pose.z < upper_bound.z && relative_pose.z > lower_bound.z) {
                return true;
        } else {
                return false;
        }
}

/*
 * @brief Check whether relative positions under tolerance
 * @param tolerance: tolerance in x, y, z direction;
 * @param relative_pose: relative position in x, y, z direction
 * @return bool
 */
bool under_tolerance(const geometry_msgs::Point& tolerance, const geometry_msgs::Point& relative_pose) {
        if ((std::abs(relative_pose.x) < tolerance.x) && (std::abs(relative_pose.y) < tolerance.y) \
            && (std::abs(relative_pose.z) < tolerance.z)) {
                return true;
        } else {
                return false;
        }
}

/*
 * @brief Gripper Control: Enable Gripper when it closes bins; disable gripper when it closes tray
 */
void grasp_bin6() {

        // Listen to tf
        tf::TransformListener listener;     // initialize tf listener
        geometry_msgs::Point relative_pose_bin6;    ///< the relative position between gripper and bin
        tf::StampedTransform transform_bin6;    // initialize transform between gripper and bin6
        listener.waitForTransform("/bin6_frame","/vacuum_gripper_link",ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/bin6_frame","/vacuum_gripper_link",ros::Time(0),transform_bin6);
        relative_pose_bin6.x = transform_bin6.getOrigin().x();
        relative_pose_bin6.y = transform_bin6.getOrigin().y();
        relative_pose_bin6.z = transform_bin6.getOrigin().z();

        //ROS_INFO_STREAM("relative pose to bin6: x = " << relative_pose_bin6.x);
        // ROS_INFO_STREAM("relative pose to bin6: y = " << relative_pose_bin6.y);
        //ROS_INFO_STREAM("relative pose to bin6: z = " << relative_pose_bin6.z);

        // set up tolerance
        geometry_msgs::Point tolerance;
        tolerance.x = 0.25;
        tolerance.y = 0.25;
        tolerance.z = 0.1;

        // if the relative position under tolerance enable the vacuum gripper
        if (under_tolerance(tolerance, relative_pose_bin6)) {
                grasp_kit();
        }
}
/*
 * @brief Gripper Control: Enable Gripper when it closes bins; disable gripper when it closes tray
 */
void grasp_bin7() {

        // Listen to tf
        tf::TransformListener listener;       // initialize tf listener
        geometry_msgs::Point relative_pose_bin7;      ///< the relative position between gripper and bin
        tf::StampedTransform transform_bin7;      // initialize transform between gripper and bin6
        listener.waitForTransform("/bin7_frame","/vacuum_gripper_link",ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/bin7_frame","/vacuum_gripper_link",ros::Time(0),transform_bin7);
        relative_pose_bin7.x = transform_bin7.getOrigin().x();
        relative_pose_bin7.y = transform_bin7.getOrigin().y();
        relative_pose_bin7.z = transform_bin7.getOrigin().z();

        //  ROS_INFO_STREAM("relative pose to bin7: x = " << relative_pose_bin7.x);
        // ROS_INFO_STREAM("relative pose to bin7: y = " << relative_pose_bin7.y);
        // ROS_INFO_STREAM("relative pose to bin7: z = " << relative_pose_bin7.z);

        // set up tolerance
        geometry_msgs::Point tolerance;
        tolerance.x = 0.25;
        tolerance.y = 0.25;
        tolerance.z = 0.1;

        // if the relative position under tolerance enable the vacuum gripper
        if (under_tolerance(tolerance, relative_pose_bin7)) {
                grasp_kit();
        }
}
/*
 * @brief Place kit on tray 1
 */
void place_kit_tray1() {
        // Listen to tf
        tf::TransformListener listener;             // initialize tf listener
        geometry_msgs::Point relative_pose_tray1;           ///< the relative position between gripper and bin
        tf::StampedTransform transform_tray1;           // initialize transform between gripper and bin6
        listener.waitForTransform("/agv1_load_point_frame","/vacuum_gripper_link",ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/agv1_load_point_frame","/vacuum_gripper_link",ros::Time(0),transform_tray1);
        relative_pose_tray1.x = transform_tray1.getOrigin().x();
        relative_pose_tray1.y = transform_tray1.getOrigin().y();
        relative_pose_tray1.z = transform_tray1.getOrigin().z();

        // ROS_INFO_STREAM("relative pose to tray1: x = " << relative_pose_tray1.x);
        // ROS_INFO_STREAM("relative pose to tray1: y = " << relative_pose_tray1.y);
        // ROS_INFO_STREAM("relative pose to tray1: z = " << relative_pose_tray1.z);


        // set up tolerance
        geometry_msgs::Point tolerance;
        tolerance.x = 0.4;
        tolerance.y = 0.3;
        tolerance.z = 0.4;

        // if the relative position under tolerance enable the vacuum gripper
        if (under_tolerance(tolerance, relative_pose_tray1)) {
                ros::Duration(0.1).sleep();
                release_kit();
        }

}

/*
 * @brief Place kit on tray 1
 */
void place_kit_tray2() {
        // Listen to tf
        tf::TransformListener listener;             // initialize tf listener
        geometry_msgs::Point relative_pose_tray2;           ///< the relative position between gripper and bin
        tf::StampedTransform transform_tray2;           // initialize transform between gripper and bin6
        listener.waitForTransform("/agv2_load_point_frame","/vacuum_gripper_link",ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("/agv2_load_point_frame","/vacuum_gripper_link",ros::Time(0),transform_tray2);
        relative_pose_tray2.x = transform_tray2.getOrigin().x();
        relative_pose_tray2.y = transform_tray2.getOrigin().y();
        relative_pose_tray2.z = transform_tray2.getOrigin().z();

        // ROS_INFO_STREAM("relative pose to tray2: x = " << relative_pose_tray2.x);
        // ROS_INFO_STREAM("relative pose to tray2: y = " << relative_pose_tray2.y);
        // ROS_INFO_STREAM("relative pose to tray2: z = " << relative_pose_tray2.z);


        // set up tolerance
        geometry_msgs::Point tolerance;
        tolerance.x = 0.4;
        tolerance.y = 0.4;
        tolerance.z = 1;

        // if the relative position under tolerance enable the vacuum gripper
        if (under_tolerance(tolerance, relative_pose_tray2)) {
                release_kit();
        }

}

private:
std::string competition_state_;
double current_score_;
ros::Publisher joint_trajectory_publisher_;
std::vector<osrf_gear::Order> received_orders_;
sensor_msgs::JointState current_joint_states_;
bool has_been_zeroed_;
bool gripper_state_attatch_ = false;
bool order_0_= false; bool order_1_ = false; bool order_2_ = false;
bool order_3_ = false; bool order_4_ = false;
ros::ServiceClient gripper_service;
ros::ServiceClient tray_delivery_;
int num_tools_in_tray_;
float duration_time_ = 0.6;
std::vector<int> pass_order0 {0,0};
std::vector<int> pass_order1 {0,0};
std::vector<int> pass_order2 {0,0};
std::vector<int> pass_order3 {0,0};
std::vector<int> pass_order4 {0,0};
};

void proximity_sensor_callback(const sensor_msgs::Range::ConstPtr & msg) {
        if ((msg->max_range - msg->range) > 0.01) { // If there is an object in proximity.
                ROS_INFO_THROTTLE(1, "Proximity sensor sees something.");
        }
}

void laser_profiler_callback(const sensor_msgs::LaserScan::ConstPtr & msg) {
        size_t number_of_valid_ranges = std::count_if(
                msg->ranges.begin(), msg->ranges.end(), [] (int i){return std::isfinite(i); });
        if (number_of_valid_ranges > 0) {
                ROS_INFO_THROTTLE(1, "Laser profiler sees something.");
        }
}



int main(int argc, char ** argv) {
        // Last argument is the default name of the node.
        ros::init(argc, argv, "ariac_example_node");

        ros::NodeHandle node;

        // Instance of custom class from above.
        MyCompetitionClass comp_class(node);

        ros::Rate rate(10.0);
        //while(ros::ok()) {

        // Subscribe to the '/ariac/current_score' topic.
        ros::Subscriber current_score_subscriber = node.subscribe(
                "/ariac/current_score", 10,
                &MyCompetitionClass::current_score_callback, &comp_class);

        // Subscribe to the '/ariac/competition_state' topic.
        ros::Subscriber competition_state_subscriber = node.subscribe(
                "/ariac/competition_state", 10,
                &MyCompetitionClass::competition_state_callback, &comp_class);


        // Subscribe to the '/ariac/orders' topic.
        ros::Subscriber orders_subscriber = node.subscribe(
                "/ariac/orders", 10,
                &MyCompetitionClass::order_callback, &comp_class);


        // Subscribe to the '/ariac/joint_states' topic.
        /*   ros::Subscriber joint_state_subscriber = node.subscribe(
                  "/ariac/joint_states", 10,
                  &MyCompetitionClass::joint_state_callback, &comp_class);
         */

        // Subscribe to the '/ariac/proximity_sensor_1' topic.
        ros::Subscriber proximity_sensor_subscriber = node.subscribe(
                "/ariac/proximity_sensor_1", 10, proximity_sensor_callback);


        // Subscribe to the '/ariac/break_beam_1_change' topic.
        ros::Subscriber break_beam_subscriber = node.subscribe(
                "/ariac/break_beam_1_change", 10,
                &MyCompetitionClass::break_beam_callback, &comp_class);

        // Subscribe to the '/ariac/logical_camera_1' topic.
        ros::Subscriber logical_camera_subscriber = node.subscribe(
                "/ariac/logical_camera_1", 10,
                &MyCompetitionClass::logical_camera_callback, &comp_class);

        // Subscribe to the '/ariac/laser_profiler_1' topic.
        ros::Subscriber laser_profiler_subscriber = node.subscribe(
                "/ariac/laser_profiler_1", 10, laser_profiler_callback);

        // Subscribe to the '/ariac/logical_camera_2' topic.
        // Subscribe the parts number on the tray
        ros::Subscriber Tray_logical_camera_subscriber = node.subscribe(
                "/ariac/logical_camera_2", 10,
                &MyCompetitionClass::tray_logical_camera_callback, &comp_class);

        // Subscribe to the '/ariac/gripper/state' topic.
        ros::Subscriber gripper_state_attatch = node.subscribe("/ariac/gripper/state",10,
                                                               &MyCompetitionClass::gripper_state_attatch_callback,&comp_class);

        tf::TransformListener listener;
        //ROS_INFO("Setup complete.");
        start_competition(node);


        ros::ServiceClient client = node.serviceClient<move_arm::Pick>("/move_arm/toPose");
        move_arm::Pick Movesrv;
        /*    ROS_INFO_STREAM("size of this shit list "<< comp_class.pick_place_list.size());
            if(!comp_class.pick_place_list.empty()){
              ROS_INFO_STREAM("goal position x is "<< comp_class.pick_place_list[0].second.position.x);
            }*/
        //  Movesrv.request.pose.position.x =-0.2; // comp_class.pick_place_list[0].second.position.x;
        //  Movesrv.request.pose.position.y =0.33; // comp_class.pick_place_list[0].second.position.y;
        //  Movesrv.request.pose.position.z =0.745; // comp_class.pick_place_list[0].second.position.z;
        //  Movesrv.request.mode = 1;
        //ROS_INFO_STREAM("goal position x is "<< comp_class.pick_place_list[0].second.position.x);
        //Movesrv.request.pose.position.x = comp_class.pick_place_list[0].second.position.x;
        //Movesrv.request.pose.position.y = comp_class.pick_place_list[0].second.position.y;
        //Movesrv.request.pose.position.z = comp_class.pick_place_list[0].second.position.z;
        //Movesrv.request.mode = 2;
        //client.call(Movesrv);
        std::vector<std::vector<float> > bin = {{-0.2,0.33,0.745},{-0.4,0.13,0.724},{-0.1,-0.335,0.745},{-0.233,-0.468,0.745},{-0.36,-0.335,0.745},{0.4,3.15,0.75},{-0.4,0.33,0.75},{-0.2,0.13,0.75},{-0.5,-0.46,0.75},{-0.5,-0.6,0.75},{-0.1,-0.6,0.75},{-0.07,0.13,0.75}};
        while(ros::ok()) {
                if(!comp_class.order_pose_list.empty()) {
                        ROS_INFO_STREAM("In mian file there are  " << comp_class.order_pose_list.size()<<" objects\n" );
                        std::vector<geometry_msgs::Pose> world_pose_list;
                        for(int i = 0; i <comp_class.order_pose_list.size(); i++) {
                                comp_class.orderPoseBroadcaster(comp_class.order_pose_list[i],i);

                                geometry_msgs::Pose world_pose = comp_class.desired_part_pose(listener,i);
                                //  if (world_pose.position.x != 0)
                                world_pose_list.push_back(world_pose);

                        }
                        //  comp_class.orderPoseBroadcaster(comp_class.order_pose_list[1],1);

                        //  geometry_msgs::Pose world_pose0 = comp_class.desired_part_pose(listener,1);
                        /*      ROS_INFO_STREAM("First x position is  " << world_pose_list[0].position.x);
                              ROS_INFO_STREAM("First y position is  " << world_pose_list[0].position.y);
                              ROS_INFO_STREAM("First z position is  " << world_pose_list[0].position.z);
                              ROS_INFO_STREAM("orientation x is  " << world_pose_list[0].orientation.x);
                              ROS_INFO_STREAM("orientation y is  " << world_pose_list[0].orientation.y);
                              ROS_INFO_STREAM("orientation z is  " << world_pose_list[0].orientation.z);
                              ROS_INFO_STREAM("orientation w is  " << world_pose_list[0].orientation.w);*/
                        for(int x = 7; x< 12; x+=4) {
                                if(world_pose_list[0].position.x != 0) {
                                        ROS_INFO_STREAM("x is  " << x);
                                        ROS_INFO_STREAM("First x position is  " << bin[x][0]);
                                        ROS_INFO_STREAM("First y position is  " << bin[x][1]);
                                        Movesrv.request.pose.position.x = bin[x][0]; // comp_class.pick_place_list[0].second.position.x;
                                        Movesrv.request.pose.position.y = bin[x][1];      // comp_class.pick_place_list[0].second.position.y;
                                        Movesrv.request.pose.position.z = 0.745;      // comp_class.pick_place_list[0].second.position.z;
                                        Movesrv.request.mode = 1;
                                        client.call(Movesrv);
                                        if(x < 6) {
                                                Movesrv.request.pose.position.x = world_pose_list[x].position.x; // comp_class.pick_place_list[0].second.position.x;
                                                Movesrv.request.pose.position.y = world_pose_list[x].position.y; // comp_class.pick_place_list[0].second.position.y;
                                                Movesrv.request.pose.position.z = 0.8; // comp_class.pick_place_list[0].second.position.z;
                                                if (x == 5) {
                                                        Movesrv.request.pose.position.x = world_pose_list[2].position.x; // comp_class.pick_place_list[0].second.position.x;
                                                        Movesrv.request.pose.position.y = world_pose_list[2].position.y; // comp_class.pick_place_list[0].second.position.y;
                                                        Movesrv.request.pose.position.z = 0.8;
                                                        Movesrv.request.pose.orientation.z = -0.2;

                                                }
                                                if(x == 0 ||x == 1)
                                                        Movesrv.request.pose.orientation.z = -0.788;
                                                if(x == 2 || x == 3 ||x == 4)
                                                        Movesrv.request.pose.orientation.z = 0;
                                        }
                                        else {
                                                if (x == 11) {
                                                        Movesrv.request.pose.position.x = world_pose_list[6].position.x;
                                                        Movesrv.request.pose.position.y = -world_pose_list[6].position.y;
                                                }
                                                else {
                                                        Movesrv.request.pose.position.x = world_pose_list[x-1].position.x;
                                                        Movesrv.request.pose.position.y = -world_pose_list[x-1].position.y;
                                                }
                                                //ROS_INFO_STREAM("Second order:First x position is  " << world_pose_list[x-1].position.x);
                                                //ROS_INFO_STREAM("Second order:First y position is  " << world_pose_list[x-1].position.y);

                                                Movesrv.request.pose.position.z = 0.8; // comp_class.pick_place_list[0].second.position.z;
                                                if(x == 6 || x == 7)
                                                        Movesrv.request.pose.orientation.z = -0.788;
                                                else if (x == 11)
                                                        Movesrv.request.pose.orientation.z = -0.788;
                                                else
                                                        Movesrv.request.pose.orientation.z = 0.0;
                                        }

                                        Movesrv.request.mode = 2;
                                        client.call(Movesrv);
                                }
                                else {
                                        ROS_INFO_STREAM("world_pose is invalid");
                                        continue;
                                }

                        }
                }

                ros::spinOnce();   // This executes callbacks on new data until ctrl-c.
                rate.sleep();
        }
        return 0;
}
