

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Empty.h"
#include <osrf_gear/VacuumGripperState.h>
#include <tf/transform_listener.h>
#include <osrf_gear/VacuumGripperControl.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>

#include <move_arm/Pick.h>

//gripper offset
#include "tf2_msgs/TFMessage.h"
#include "part_perception/Part_Offset_Gripper.h"
#include "part_perception/Inventory_Predication.h"

#include <vector>
#include "math.h"

#include <osrf_gear/AGVControl.h>

//TF offset calculation technique
#include <tf/transform_broadcaster.h>

#include <ros/callback_queue.h>
#include <ros/spinner.h>

tf::StampedTransform gripper_transform;
tf::StampedTransform tray_transform;
geometry_msgs::PoseStamped gripper_target;
geometry_msgs::Pose gripper_target_world;
geometry_msgs::Pose target_pose3;
osrf_gear::VacuumGripperControl srv;
std::vector<geometry_msgs::Pose> waypoints;
ros::Publisher joint_trajectory_publisher_;
float duration_time_ = 0.2;

sensor_msgs::JointState current_joint_states_;
trajectory_msgs::JointTrajectory msg;



moveit::planning_interface::MoveGroupInterface *group;
ros::ServiceClient client;
ros::ServiceClient part_perception_client;
ros::ServiceClient belt_query_client;

//arm state variables
bool grabbing = false;
bool enabled = false;
bool attached = false;

double gripper_to_base_offset = -0.169946;


tf::Transform part_target_to_world;
tf::Transform gripper_target_to_part;




/// Start the competition by waiting for and then calling the start ROS Service.
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


void generate_gripper_target(float dx, float dy, float dz)
{
								gripper_target.header.frame_id = "world";
								gripper_target.pose.position.x = gripper_transform.getOrigin().x();
								gripper_target.pose.position.y = gripper_transform.getOrigin().y();
								gripper_target.pose.position.z = gripper_transform.getOrigin().z();
								gripper_target.pose.orientation.w = 0.707;
								gripper_target.pose.orientation.x = 0.0;
								gripper_target.pose.orientation.y = 0.707;
								gripper_target.pose.orientation.z = 0.0;


}



void pne(float t, float stepSize)
{
								group->setMaxVelocityScalingFactor(1.0);
								group->setMaxAccelerationScalingFactor(1.0);
								group->setPlanningTime(t);

								moveit_msgs::RobotTrajectory trajectory;
								double fraction = group->computeCartesianPath(waypoints,
																																																						stepSize, // eef_step
																																																						0.0, // jump_threshold
																																																						trajectory);

								ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
																	fraction * 100.0);


								moveit::planning_interface::MoveGroup::Plan my_plan;
								my_plan.trajectory_ = trajectory;
								group->execute(my_plan);
}






void check_stable(float tolerance)

{
								int i = 0;
								bool isStable = false;
								while (!isStable) {
																isStable = true;
																ros::spinOnce();
																for (int j = 0; j < 6; j++) {
																								// if(!(msg.points[0].positions[j]+tolerance > current_joint_states_.position[0] && msg.points[0].positions[j]-tolerance < current_joint_states_.position[0])){
																								//  isStable = false;
																								//  //ROS_INFO("position OK");
																								// }
																								if (std::abs(current_joint_states_.velocity[j]) > tolerance) {
																																//ROS_INFO("speed  not ok: %d : %d",j,current_joint_states_.velocity[j]);
																																isStable = false;
																								}

																}
																i++;
																if (i > 1000) {
																								isStable = true;
																}
																//ROS_INFO_STREAM("waiting for stable"<<current_joint_states_);
																ros::Duration(0.1).sleep();
								}
								//ROS_INFO_STREAM("done waiting"<<current_joint_states_);
								//ROS_INFO_STREAM("final i: "<<i);

}

void move_joints(float p0, float p1, float p2, float p3, float p4, float p5, float p6, float duration) {
								msg.joint_names.clear();
								msg.joint_names.push_back("elbow_joint");
								msg.joint_names.push_back("linear_arm_actuator_joint");
								msg.joint_names.push_back("shoulder_lift_joint");
								msg.joint_names.push_back("shoulder_pan_joint");
								msg.joint_names.push_back("wrist_1_joint");
								msg.joint_names.push_back("wrist_2_joint");
								msg.joint_names.push_back("wrist_3_joint");

								msg.points.resize(1);
								msg.points[0].positions = {p0, p1, p2, p3, p4, p5, p6};
								// How long to take getting to the point (floating point seconds).
								msg.points[0].time_from_start = ros::Duration(duration);
								msg.header.stamp = ros::Time::now() + ros::Duration();
								//ROS_INFO_STREAM("Sending command:\n" << msg);
								joint_trajectory_publisher_.publish(msg);
								ros::spinOnce();
}



void move_to(float x, float y, float z, float dyaw) {
								if (y > 3.45) {
																y = 3.45;
								}

								if (y < -3.45) {
																y = -3.45;
								}

								if (x > 3.45) {
																x = 3.45;
								}

								if (x < -3.45) {
																x = -3.45;
								}


								check_stable(0.03);


								tf::TransformListener listener;
								listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
								listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
								ros::spinOnce();
								waypoints.clear();
								generate_gripper_target(0, 0, 0);
								geometry_msgs::Pose target_pose3 = gripper_target.pose;

								double safeHeight = 1.0f;
								double safeTrackFront = 1.0f;
								//before moving in x,y, make sure z is above certain height
								if (gripper_target.pose.position.z < safeHeight) {

																waypoints.push_back(target_pose3);
																target_pose3.position.z = safeHeight;
																waypoints.push_back(target_pose3);
																pne(15.0, 0.01);
																ros::spinOnce();
								}
								int i = 0;

								while (gripper_target.pose.position.z < safeHeight && i < 100) {
																ros::spinOnce();
																sleep(0.1);
																i++;
								}



								//move from mid to agv1 direction
								i = 0;
								if (gripper_target.pose.position.y < 1.5 && gripper_target.pose.position.y > -1.5 && y > 1.5) {


																// Create a message to send.
																move_joints(2.1,
																												0,
																												-1.57,
																												3.14,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);

																check_stable(0.03);

																// Create a message to send.
																move_joints(2.1,
																												0,
																												-1.57,
																												1.57,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);
																i = 0;
																while (!(1.55 < current_joint_states_.position[3] && 1.59 > current_joint_states_.position[3]) && i < 1000) {
																								//ROS_INFO("waitinf for arm move");
																								ros::spinOnce();
																								sleep(0.1);
																								i++;
																}


																move_joints(current_joint_states_.position[0],
																												1.8,
																												current_joint_states_.position[2],
																												current_joint_states_.position[3],
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);


																check_stable(0.03);


																move_joints(1.13,
																												2.0,
																												-0.5,
																												1.57,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);


																check_stable(0.03);
								}


								//back from agv1 to mid
								if (gripper_target.pose.position.y > 1.5 && y < 1.5 && y > -1.5)
								{
																move_joints(2.1,
																												0,
																												-1.57,
																												1.57,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);
																i = 0;
																while (current_joint_states_.position[1] > 0.5 && i < 1000) {
																								//ROS_INFO("waitinf for arm move");
																								ros::spinOnce();
																								sleep(0.1);
																								i++;
																}


																check_stable(0.05);

																move_joints(current_joint_states_.position[0],
																												0,
																												current_joint_states_.position[2],
																												3.14,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);


																check_stable(0.03);

								}


								/*
								   *===================
								   *2nd bin holding right
								   *elbow_joint 2.25
								   *linear_arm_actuator_joint 0.5
								   *shoulder_lift_joint -0.75
								   *shoulder_pan_joint 3.14159
								   *wrist_1_joint 3.22
								   *wrist_2_joint -1.57
								   *wrist_3_joint 0.00
								 */

								//from camera to avg2
								if (gripper_target.pose.position.y > 1.5 && y < -1.5)
								{
																ros::spinOnce();
																move_joints(2.1,
																												1.0,
																												-1.57,
																												1.57,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);
																ros::spinOnce();
																move_joints(2.1,
																												0,
																												-1.57,
																												3.14,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												0.1);
																i = 0;
																while (current_joint_states_.position[1] > 0.5 && i < 1000) {
																								//ROS_INFO("waiting for arm move");
																								ros::spinOnce();
																								sleep(0.1);
																								i++;
																}





																ros::spinOnce();
																check_stable(0.05);
																ros::spinOnce();
																move_joints(2.1,
																												0,
																												-1.57,
																												4.71,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												0.1);

																i = 0;
																while (!(4.69 < current_joint_states_.position[3] && 4.73 > current_joint_states_.position[3]) && i < 1000) {
																								//ROS_INFO("waitinf for arm move");
																								sleep(0.1);
																								ros::spinOnce();
																								i++;
																}

																ros::spinOnce();
																check_stable(0.05);
																ros::spinOnce();

																move_joints(1.13,
																												-1.8,
																												-0.5,
																												4.71,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												0.5);


																check_stable(0.03);
																ros::spinOnce();

								}

								//go to agv2
								if (gripper_target.pose.position.y < 1.5 && gripper_target.pose.position.y > -1.5 && y < -1.5) {


																// Create a message to send.
																move_joints(2.1,
																												0,
																												-1.57,
																												3.14,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);

																check_stable(0.03);

																// Create a message to send.
																move_joints(2.1,
																												0,
																												-1.57,
																												4.71,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);
																i = 0;
																while (!(4.69 < current_joint_states_.position[3] && 4.73 > current_joint_states_.position[3]) && i < 1000) {
																								//ROS_INFO("waitinf for arm move");
																								sleep(0.1);
																								ros::spinOnce();
																								i++;
																}


																move_joints(current_joint_states_.position[0],
																												-1.8,
																												current_joint_states_.position[2],
																												current_joint_states_.position[3],
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);


																check_stable(0.03);


																move_joints(1.13,
																												-2.0,
																												-0.5,
																												4.71,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);


																check_stable(0.03);
								}

								//back from agv2
								if (gripper_target.pose.position.y < -1.5 && y < 1.5 && y > -1.5)
								{
																move_joints(2.1,
																												0,
																												-1.57,
																												4.71,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);

																while (current_joint_states_.position[1] < -0.5) {
																								//ROS_INFO("waitinf for arm move");
																								ros::spinOnce();
																}


																check_stable(0.05);

																move_joints(current_joint_states_.position[0],
																												0,
																												current_joint_states_.position[2],
																												3.14,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												1);


																check_stable(0.03);

								}

								if (gripper_target.pose.position.y > 2 && y > 2 ) {
																ros::spinOnce();
																move_joints(current_joint_states_.position[0],
																												current_joint_states_.position[1],
																												current_joint_states_.position[2],
																												1.57,
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												0.3);

																sleep(2.0);

																check_stable(0.03);

								}







								check_stable(0.03);
								ros::spinOnce();

								listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
								listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
								ros::spinOnce();
								waypoints.clear();
								generate_gripper_target(0, 0, 0);
								target_pose3 = gripper_target.pose;
								waypoints.push_back(target_pose3);

								//do rotation
								tf::Quaternion q = tf::createQuaternionFromRPY(0, 1.57, dyaw);
								q.normalize();
								ROS_INFO("Quaternion: %f, %f, %f, %f", q[0], q[1], q[2], q[3]);

								target_pose3.orientation.w = q[3];
								target_pose3.orientation.x = q[0];
								target_pose3.orientation.y = q[1];
								target_pose3.orientation.z = q[2];


								//maintain height before dropping to avoid tray edge collision
								if (z < safeHeight) {
																target_pose3.position.x = x;
																target_pose3.position.y = y;
																target_pose3.position.z = safeHeight;
																waypoints.push_back(target_pose3);

								}

								target_pose3.position.x = x;
								target_pose3.position.y = y;
								target_pose3.position.z = z;
								waypoints.push_back(target_pose3);

								pne(15.0, 0.01);


}




/*
   *===================
   *pre belt pickup holding position
   *elbow_joint 1.65
   *linear_arm_actuator_joint 0.00
   *shoulder_lift_joint -0.69
   *shoulder_pan_joint 0.00
   *wrist_1_joint 3.73
   *wrist_2_joint -1.51
   *wrist_3_joint 0.00
 */

void go_to_belt(float y) {

								tf::TransformListener listener;
								listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
								listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
								ros::spinOnce();

								y = y + gripper_to_base_offset;

								if (gripper_transform.getOrigin().y() > 1.5 || gripper_transform.getOrigin().y() < -1.5) {
																move_joints(2.1,
																												0,
																												-1.57,
																												current_joint_states_.position[3],
																												current_joint_states_.position[4],
																												current_joint_states_.position[5],
																												current_joint_states_.position[6],
																												0.1);
																while (!(-0.1 < current_joint_states_.position[1] && 0.1 > current_joint_states_.position[1])) {
																								//ROS_INFO("waitinf for arm move");
																								ros::spinOnce();
																}

								}



								//ROS_INFO("go to belt move");
								// // rotate towards
								move_joints(2.1,
																				0,
																				-1.57,
																				3.14,
																				current_joint_states_.position[4],
																				current_joint_states_.position[5],
																				current_joint_states_.position[6],
																				0.1);
								sleep(2.0);

								move_joints(2.1,
																				0,
																				-1.57,
																				0,
																				current_joint_states_.position[4],
																				current_joint_states_.position[5],
																				current_joint_states_.position[6],
																				0.1);
								sleep(1.0);
								//wait for arm to go to belt direction before move
								while (!(1.4 > current_joint_states_.position[3] ) ) {
																//ROS_INFO("waitinf for arm move");
																sleep(0.1);
																ros::spinOnce();
								}

								move_joints(1.65,
																				y,
																				-1.2,
																				0.00,
																				3.73,
																				-1.51,
																				0.00,
																				0.1);
								check_stable(0.05);


								int i = 0;
								while (!(-0.3 < current_joint_states_.position[3] && 0.3 > current_joint_states_.position[3]) && i < 1000) {
																//ROS_INFO("waitinf for arm move");
																sleep(0.1);
																ros::spinOnce();
																i++;
								}

								srv.request.enable = false;
								client.call(srv);
								ros::spinOnce();


								move_joints(1.65,
																				y,
																				-0.69,
																				0.00,
																				3.73,
																				-1.51,
																				0.00,
																				0.5);
								check_stable(0.05);


}

void fast_pick_up() {
								srv.request.enable = true;
								client.call(srv);
								ros::spinOnce();

								move_joints(1.69,
																				current_joint_states_.position[1],
																				current_joint_states_.position[2],
																				current_joint_states_.position[3],
																				current_joint_states_.position[4],
																				current_joint_states_.position[5],
																				current_joint_states_.position[6],
																				0.1);
								sleep(1.0);
								move_joints(1.60,
																				current_joint_states_.position[1],
																				current_joint_states_.position[2],
																				current_joint_states_.position[3],
																				current_joint_states_.position[4],
																				current_joint_states_.position[5],
																				current_joint_states_.position[6],
																				0.1);

}


void fast_pick_up_at_time(double secs) {
								bool isTime = false;
								double now_secs = 0;
								secs = secs - 0.1;

								while (!isTime) {
																ros::spinOnce();
																now_secs = ros::Time::now().toSec();
																if (now_secs > secs) {
																								isTime = true;
																}
																sleep(0.1);
																ROS_INFO("waiting time: %f  now: %f", secs, now_secs);
								}


								srv.request.enable = true;
								client.call(srv);
								ros::spinOnce();

								move_joints(1.68,
																				current_joint_states_.position[1],
																				-0.665,
																				0.00,
																				3.72,
																				-1.57,
																				current_joint_states_.position[6],
																				0.1);
								sleep(1.0);
								move_joints(1.55,
																				0,
																				-1.5,
																				current_joint_states_.position[3],
																				current_joint_states_.position[4],
																				-1.57,
																				current_joint_states_.position[6],
																				0.1);

}


/*
   *===================
   *2nd bin holding right
   *elbow_joint 2.25
   *linear_arm_actuator_joint 0.5
   *shoulder_lift_joint -0.75
   *shoulder_pan_joint 3.14159
   *wrist_1_joint 3.22
   *wrist_2_joint -1.57
   *wrist_3_joint 0.00
 */
/*
   *===================
   *2nd bin holding right
   *elbow_joint 2.25
   *linear_arm_actuator_joint 0.5
   *shoulder_lift_joint -0.75
   *shoulder_pan_joint 3.14159
   *wrist_1_joint 3.22
   *wrist_2_joint 0.00
   *wrist_3_joint 0.00
 */

void flip_part() {
								ros::spinOnce();

								//raise shoulder for clearance
								move_joints(-1.62,
																				2.0,
																				-1.57,
																				3.14,
																				5.78,
																				0,
																				0.0,
																				0.5);
								sleep(1.0);

								//flip facing the part to down the belt
								move_joints(-1.62,
																				2.0,
																				-2.45,
																				3.14,
																				5.78,
																				0,
																				0.0,
																				0.75);
								check_stable(0.05);
								sleep(1.0);

								srv.request.enable = false;
								client.call(srv);
								ros::spinOnce();
								sleep(2.0);

								//raise shoulder
								move_joints(-1.62,
																				2.0,
																				-1.57,
																				3.14,
																				5.78,
																				0,
																				0.0,
																				0.5);
								sleep(1.0);
								ros::spinOnce();

								//move down and flip
								move_joints(1.62,
																				0,
																				-1.57,
																				0,
																				3.22,
																				0,
																				0.00,
																				0.1);
								sleep(1.0);

								move_joints(1.62,
																				0,
																				-1.57,
																				0,
																				3.22,
																				0,
																				0.00,
																				0.3);
								sleep(1.0);

								move_joints(1.62,
																				0,
																				-0.69,
																				0,
																				4.0,
																				0,
																				0.00,
																				0.5);
								sleep(3.0);

								srv.request.enable = true;
								client.call(srv);
								ros::spinOnce();

								move_joints(1.62,
																				2.0,
																				-0.69,
																				0,
																				4.0,
																				0,
																				0.00,
																				15.0);

								while (!attached) {
																sleep(0.1);
																ros::spinOnce();
								}

								move_joints(1.62,
																				0,
																				-1.57,
																				0,
																				4.0,
																				0,
																				0.00,
																				0.1);
								sleep(1.0);
								/*
								   *===================
								   *2nd bin holding right
								   *elbow_joint 2.25
								   *linear_arm_actuator_joint 0.5
								   *shoulder_lift_joint -0.75
								   *shoulder_pan_joint 3.14159
								   *wrist_1_joint 3.22
								   *wrist_2_joint -1.57
								   *wrist_3_joint 0.00
								 */

								move_joints(1.62,
																				2.0,
																				-0.75,
																				0,
																				3.34,
																				-1.57,
																				0.00,
																				0.3);
								sleep(1.0);

								move_joints(1.62,
																				2.0,
																				-0.75,
																				0,
																				3.34,
																				-1.57,
																				0.00,
																				0.3);
								sleep(1.0);
								check_stable(0.05);



}


void go_to_camera() {
								tf::TransformListener listener;
								listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
								listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
								ros::spinOnce();
								waypoints.clear();
								generate_gripper_target(0, 0, 0);
								geometry_msgs::Pose target_pose3 = gripper_target.pose;

								double safeHeight = 1.0f;
								double safeTrackFront = 1.0f;
								//before moving in x,y, make sure z is above certain height
								if (gripper_target.pose.position.z < safeHeight) {

																waypoints.push_back(target_pose3);
																target_pose3.position.z = safeHeight;
																waypoints.push_back(target_pose3);
																pne(15.0, 0.01);
																ros::spinOnce();
								}


								ros::spinOnce();
								move_joints(current_joint_states_.position[0],
																				0.0,
																				current_joint_states_.position[2],
																				current_joint_states_.position[3],
																				current_joint_states_.position[4],
																				current_joint_states_.position[5],
																				current_joint_states_.position[6],
																				0.1);
								sleep(2.0);
								move_joints(2.25,
																				0.0,
																				-1.57,
																				3.14159,
																				3.22,
																				-1.57,
																				0.00,
																				0.3);
								sleep(2.0);
								move_joints(2.25,
																				0.0,
																				-1.57,
																				0,
																				3.22,
																				-1.57,
																				0.00,
																				0.3);
								sleep(2.0);
								move_joints(1.49,
																				2.0,
																				-1.12,
																				0,
																				4.30,
																				-1.57,
																				0.00,
																				0.3);
								sleep(1.0);

								move_joints(1.49,
																				2.0,
																				-1.12,
																				0,
																				4.30,
																				-1.57,
																				0.00,
																				0.5);
								sleep(1.0);
								check_stable(0.05);


								// listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
								// listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
								// ros::spinOnce();
								// waypoints.clear();
								// generate_gripper_target(0, 0, 0);
								// target_pose3 = gripper_target.pose;
								// waypoints.push_back(target_pose3);
								// target_pose3.position.z = 1.4f;
								// waypoints.push_back(target_pose3);
								// pne(15.0, 0.01);
								// ros::spinOnce();

}





void gripper_callback(const osrf_gear::VacuumGripperState msg)
{
								enabled = msg.enabled;
								attached = msg.attached;

}

bool check_release(float x, float y, float z, float tolerance) {
								tf::TransformListener listener;
								listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
								listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
								ros::spinOnce();
								x = std::abs(gripper_transform.getOrigin().x() - x);
								y = std::abs(gripper_transform.getOrigin().y() - y);
								z = std::abs(gripper_transform.getOrigin().z() - z);


								if (x <= tolerance && y <= tolerance && z <= tolerance) {
																ROS_INFO("tolerance passed: %f ,%f ,%f", x, y, z);
																return true;
								}



								return false;

}



// void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
//  r1 = msg->pose.pose.position.x;
//  r2 = msg->pose.pose.position.y;
//  r3 = msg->pose.pose.position.z;
//  tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
//  tf::Matrix3x3 m(q);
//  double roll, pitch;
//  m.getRPY(roll, pitch, yaw);
//  yaw = (yaw/(pi))*180.0;
//  if(yaw <0) yaw+= 360.0;
//  roll = (roll/(2*pi))*180.0;
//  pitch = (pitch/(2*pi))*180.0;
//     //ROS_INFO("roll pitch yaw: %f %f %f", roll, pitch, yaw);
//     //ROS_INFO("yaw: %f",yaw);

//  //v1 = msg->twist.twist.linear.x;
//  //v2 = msg->twist.twist.linear.y;
//  //v3 = msg->twist.twist.linear.z;
//  // seconds = msg->header.stamp.sec + ((double)msg->header.stamp.nsec)/1000000000;
//     // ROS_INFO("[%f]", seconds);

// }

// tf::TransformListener listener;
// tf::StampedTransform camera_transform;
// listener.waitForTransform("/world", "/logical_camera_1_frame", ros::Time(0), ros::Duration(10.0));
// listener.lookupTransform("/world", "/logical_camera_1_frame", ros::Time(0), camera_transform);
// ros::spinOnce();

// // !!!!!!!!!!!!!hard coded camera location here
// float x = 1.24;
// float y = 1.24;
// float z = 1.24;

//if (std::abs(camera_transform.getOrigin().x() - x) > 0.01 ||


//returns relative x, y, z and dyaw for gripper to accomplish
tf::Transform compute_offset_transform(geometry_msgs::Pose pose) {


								tf::Transform req_pose_transform;

								req_pose_transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
								tf::Quaternion q;
								q.setRPY(pose.orientation.x, pose.orientation.y, pose.orientation.z);
								q.normalize();
								req_pose_transform.setRotation(q);
								// br.sendTransform(tf::StampedTransform(req_pose_transform, ros::Time::now(), "/world", "part_target"));

								tf::Transform p;
								// p.translation.x = 0;
								// p.translation.y = 0;
								// p.translation.z = 0;
								// p.rotation.x = 0;
								// p.rotation.y = 0;
								// p.rotation.z = 0;
								// p.rotation.w = 1;

								ROS_INFO("Computing Offset");
								if (!part_perception_client.exists()) {
																ROS_INFO("Part Perception Service not Started");
																part_perception_client.waitForExistence();
								}

								part_perception::Part_Offset_Gripper part_perception_srv;
								part_perception_srv.request.check_part_offset = true;
								part_perception_client.call(part_perception_srv);



								if (!part_perception_srv.response.part_offset_info.transforms.empty()) {
																float x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
																float y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
																float z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;
																float camera_move_tolerance = 0.15;
																if (std::abs(x) > camera_move_tolerance || std::abs(y) > camera_move_tolerance || std::abs(z) > camera_move_tolerance) {
																								ROS_INFO("Camera has moved!!!");

																								bool ok = false;
																								int i = 0;
																								while (!ok && i < 1000) {
																																part_perception_srv.request.check_part_offset = true;
																																part_perception_client.call(part_perception_srv);
																																x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
																																y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
																																z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;
																																if (std::abs(x) < camera_move_tolerance && std::abs(y) < camera_move_tolerance && std::abs(z) < camera_move_tolerance) {
																																								ok = true;
																																}
																																ROS_INFO("Trying to perceive: %d", i);
																																ros::spinOnce();
																																i++;
																																sleep(0.3);
																								}

																}
																ROS_INFO("Part Perception: x %f y %f z %f", x,
																									y,
																									z);

																p.setOrigin(tf::Vector3(x, y, z));
																tf::Quaternion q1(part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.x,
																																		part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.y,
																																		part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.z,
																																		part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.w);
																q1.normalize();
																p.setRotation(q1);


																part_target_to_world = req_pose_transform;
																gripper_target_to_part = p;

																// br.sendTransform(tf::StampedTransform(p, ros::Time::now(), "part_target", "gripper_target"));


																// ros::Rate rate(10.0);
																// while (1) {
																//  br.sendTransform(tf::StampedTransform(req_pose_transform, ros::Time::now(), "/world", "part_target"));

																//  br.sendTransform(tf::StampedTransform(p, ros::Time::now(), "/part_target", "gripper_target"));
																//  //ROS_INFO("PUBLISHING transforms");
																//  rate.sleep();
																// }




																// ROS_INFO("Part Perception: %f",
																//          part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x);
																// tf::Quaternion q(part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.x,
																//                  part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.y,
																//                  part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.z,
																//                  part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.w);
																// tf::Matrix3x3 m(q);
																// double roll, pitch, yaw;
																// m.getRPY(roll, pitch, yaw);
																// ROS_INFO("Part Perception: x %f y %f z %f", part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x,
																//          part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y,
																//          part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z);
																// ROS_INFO("Part Perception: Roll %f Pitch %f Yaw %f", roll, pitch, yaw);

																// p.orientation.x = roll;
																// p.orientation.y = pitch;
																// p.orientation.z = yaw;

																// x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
																// y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
																// z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;


																// // p.position.x = - y * cos(yaw) + x * sin(yaw);
																// // p.position.y =  - y * sin(yaw) - x * cos(yaw);
																// // p.position.z = z * 2.0;


																// p.position.x = + y * sin(yaw)  - x * cos(yaw);
																// p.position.y =  y * cos(yaw) + x * sin(yaw);
																// p.position.z = z * 2.0;


																// ROS_INFO("offset ysin: %f", y * sin(yaw));
																// ROS_INFO("offset ycos: %f", y * cos(yaw));
																// ROS_INFO("offset xsin: %f", x * sin(yaw));
																// ROS_INFO("offset xcos: %f", x * cos(yaw));

																// //xsin xcos should be ok from top left







								} else {
																ROS_INFO("Gripper is holding nothing!");
								}

								return p;

}

// rosservice call /ariac/query_belt_part "part_type: 'gasket'
// future_time:
//   secs: 65
//   nsecs: 0"


bool query_belt_part(std::string part_type) {
								if (!belt_query_client.exists()) {
																ROS_INFO("Part Perception Service not Started");
																belt_query_client.waitForExistence();
								}

								part_perception::Inventory_Predication belt_query_srv;
								belt_query_srv.request.part_type = part_type;
								belt_query_srv.request.future_time.sec = ros::Time::now().sec + 5;
								belt_query_srv.request.future_time.nsec = ros::Time::now().nsec;
								belt_query_client.call(belt_query_srv);

								if (belt_query_srv.response.success) {
																ROS_INFO("belt query SUCCESS");

								} else {
																ROS_INFO("belt query FAILURE");
																return false;
								}



}


void tf_broadcast() {
								tf::TransformBroadcaster br;
								br.sendTransform(tf::StampedTransform(part_target_to_world, ros::Time::now(), "/world", "part_target"));
								br.sendTransform(tf::StampedTransform(gripper_target_to_part, ros::Time::now(), "/part_target", "gripper_target"));
}


bool add(move_arm::Pick::Request  &req, move_arm::Pick::Response &res)
{
								int i = 0;
								res.sum = 0;
								//ROS_INFO("move order received");
								if (req.mode == 1) {
																srv.request.enable = true;
																client.call(srv);

																ros::spinOnce();
																move_to(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0);
																i = 0;
																while (!check_release(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0.05f ) && i < 10000) {
																								//ROS_INFO("waiting for arm to arrive");
																								ros::spinOnce();
																								sleep(0.1);
																								i++;
																}
																check_stable(0.03);
																i = 0;
																while (!attached && i < 100) {
																								sleep(0.1);
																								ros::spinOnce();
																								i++;
																}
																if (i >= 100) {
																								res.sum = -1;
																								ROS_INFO("!!!!!!!!!!!Pick Failed!");
																}

								}


								if (req.mode == 2) {

																go_to_camera();



																if (std::abs(req.pose.orientation.x) > 0 || std::abs(req.pose.orientation.y) > 0) {
																								flip_part();
																}
																check_stable(0.03);
																bool drop_check_finished = false;

																if (attached == false) {
																								ROS_INFO("!!!!!!!!!!!part dropped at %f", gripper_transform.getOrigin().y());
																								res.sum = -1;
																								drop_check_finished = true;
																}
																compute_offset_transform(req.pose);
																tf::StampedTransform p;




																ros::spinOnce();
																tf::TransformListener listener;
																listener.waitForTransform("/world", "gripper_target", ros::Time(0), ros::Duration(10.0));
																listener.lookupTransform("/world", "gripper_target", ros::Time(0), p);
																ros::spinOnce();
																//spinner.stop();
																// p = compute_offset_transform();
																// p = compute_offset_transform();
																// p = compute_offset_transform();
																// float x = req.pose.position.x + p.translation.x;
																// float y = req.pose.position.y + p.translation.y;
																// float z = req.pose.position.z ; //+ p.position.z
																// float dyaw = req.pose.orientation.z + p.orientation.z + 1.57;
																float x = p.getOrigin().x();
																float y = p.getOrigin().y();
																float z = p.getOrigin().z(); //+ p.position.z
																float dyaw = req.pose.orientation.z + 1.57;

																tf::Quaternion q(p.getRotation().x(),
																																	p.getRotation().y(),
																																	p.getRotation().z(),
																																	p.getRotation().w());
																tf::Matrix3x3 m(q);
																double roll, pitch, yaw;
																m.getRPY(roll, pitch, yaw);

																move_to(x, y, z, yaw + 1.57079632679);
																// ROS_INFO("OFFSET: x: %f , y: %f, z: %f", p.translation.x, p.translation.y, p.translation.z);
																ROS_INFO("Commanded Yaw: %f", dyaw);

																//ROS_INFO("not printing");
																ros::spinOnce();
																listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
																listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
																ros::spinOnce();

																// x = std::abs(gripper_transform.getOrigin().x() - x);
																// y = std::abs(gripper_transform.getOrigin().y() - y);
																// z = std::abs(gripper_transform.getOrigin().z() - z);

																i = 0;
																while (!check_release(x, y, z, 0.005f ) && i < 10000 && !drop_check_finished) {
																								//ROS_INFO("waiting for arm to arrive");
																								ros::spinOnce();
																								listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
																								listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);

																								if (attached == false && res.sum == 0) {
																																ROS_INFO("!!!!!!!!!!!part dropped at %f", gripper_transform.getOrigin().y());
																																res.sum = -1;
																																if (gripper_transform.getOrigin().y() > 2 || gripper_transform.getOrigin().y() < -2) {
																																								ROS_INFO("Dropped on AVG, counting as OK!");
																																								res.sum = 0;
																																}
																																drop_check_finished = true;
																								}
																								sleep(0.1);
																								i++;
																}
																ros::spinOnce();
																listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
																listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);

																if (attached == false && drop_check_finished == false) {
																								ROS_INFO("!!!!!!!!!!!part dropped at %f", gripper_transform.getOrigin().y());
																								res.sum = -1;
																}
																if (drop_check_finished == false) {
																								if (gripper_transform.getOrigin().y() > 2 || gripper_transform.getOrigin().y() < -2) {
																																ROS_INFO("Dropped on AVG, counting as OK!");
																																res.sum = 0;
																								}
																}

																check_stable(0.03);


																srv.request.enable = false;
																//ROS_INFO("calling gripper release service");
																client.call(srv);

																ros::spinOnce();
																i = 0;
																while (!check_release(x, y, z, 0.02f ) && i < 10000) {
																								//ROS_INFO("waiting for arm to arrive");
																								ros::spinOnce();
																								sleep(0.1);
																								i++;
																}
																check_stable(0.03);

								}

								if (req.mode == 5) {



																go_to_camera();

								}




								if (req.mode == 3) {
																// go_to_belt(req.pose.position.y);
																// fast_pick_up_at_time(req.future_time.toSec());
																//query_belt_part(req.part_type);
																ROS_INFO("Mode 3: Looking for %s on belt", req.part_type.c_str());
																if (!belt_query_client.exists()) {
																								ROS_INFO("Part Perception Service not Started");
																								belt_query_client.waitForExistence();
																}

																part_perception::Inventory_Predication belt_query_srv;
																belt_query_srv.request.part_type = req.part_type;
																belt_query_srv.request.future_time.sec = ros::Time::now().sec + 5;
																belt_query_srv.request.future_time.nsec = ros::Time::now().nsec;
																belt_query_client.call(belt_query_srv);



																if (belt_query_srv.response.success) {
																								ROS_INFO("belt query SUCCESS");
																								if (belt_query_srv.response.parts_info.transforms[0].transform.translation.y + 2.2 < 1.8 && belt_query_srv.response.parts_info.transforms[0].transform.translation.y + 2.2 > -1.8) {
																																ROS_INFO("belt query in range");
																																go_to_belt(belt_query_srv.response.parts_info.transforms[0].transform.translation.y + 2.2);
																																fast_pick_up_at_time(belt_query_srv.request.future_time.toSec());

																																bool isTime = false;
																																double now_secs = 0;
																																double secs = belt_query_srv.request.future_time.toSec();
																																secs = secs + 1.0;


																																while (!isTime) {
																																								ros::spinOnce();
																																								now_secs = ros::Time::now().toSec();
																																								if (now_secs > secs) {
																																																isTime = true;
																																								}
																																								sleep(0.1);
																																								ROS_INFO("Check Stuck waiting time: %f  now: %f", secs, now_secs);
																																}

																																tf::TransformListener listener;
																																listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
																																listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
																																ros::spinOnce();
																																// x = std::abs(gripper_transform.getOrigin().x() - x);
																																// y = std::abs(gripper_transform.getOrigin().y() - y);
																																// z = std::abs(gripper_transform.getOrigin().z() - z);

																																if (gripper_transform.getOrigin().z() < 1.2) {
																																								ROS_INFO("STUCK ON BELT!!!! RELEASEING");
																																								srv.request.enable = false;
																																								client.call(srv);
																																								ros::spinOnce();
																																								res.sum = -1;

																																}


																								} else {
																																ROS_INFO("belt query out of range, returning -1");
																								}

																								ros::spinOnce();
																								if (attached) {
																																res.sum = 0;
																								} else {
																																res.sum = -1;
																								}

																} else {
																								ROS_INFO("belt query FAILURE");
																								res.sum = -1;
																}

								}

								if (req.mode == 4) {
																srv.request.enable = true;
																client.call(srv);
																ros::spinOnce();
																go_to_belt(0);
																double secs = ros::Time::now().sec + 5;
																fast_pick_up_at_time(secs);

																bool isTime = false;
																double now_secs = 0;
																secs = secs + 1.0;


																while (!isTime) {
																								ros::spinOnce();
																								now_secs = ros::Time::now().toSec();
																								if (now_secs > secs) {
																																isTime = true;
																								}
																								sleep(0.1);
																								ROS_INFO("Check Stuck waiting time: %f  now: %f", secs, now_secs);
																}

																tf::TransformListener listener;
																listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
																listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
																ros::spinOnce();
																// x = std::abs(gripper_transform.getOrigin().x() - x);
																// y = std::abs(gripper_transform.getOrigin().y() - y);
																// z = std::abs(gripper_transform.getOrigin().z() - z);

																if (gripper_transform.getOrigin().z() < 0.5) {
																								ROS_INFO("STUCK ON BELT!!!! RELEASEING");
																								srv.request.enable = false;
																								client.call(srv);
																								ros::spinOnce();

																}






																// srv.request.enable = true;
																// client.call(srv);

																// ros::spinOnce();
																// move_to(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0);
																// while (!check_release(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0.05f )) {
																//  //ROS_INFO("waiting for arm to arrive");
																//  ros::spinOnce();
																//  sleep(0.1);
																// }

																// move_to(req.pose.position.x, req.pose.position.y, req.pose.position.z + 0.2f, 0);
																// while (!check_release(req.pose.position.x, req.pose.position.y, req.pose.position.z + 0.2, 0.05f )) {
																//  //ROS_INFO("waiting for arm to arrive");
																//  ros::spinOnce();
																//  sleep(0.1);
																// }


																// go_to_camera();
																//compute_offset_transform();


								}


								return true;
}

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
								msg.points[0].positions = {1.56, 0.48, -1.13, 3.14, 3.58, -1.51, 0.0};
								// How long to take getting to the point (floating point seconds).
								msg.points[0].time_from_start = ros::Duration(duration_time_);
								msg.header.stamp = ros::Time::now() + ros::Duration();
								ROS_INFO_STREAM("Sending command:\n" << msg);
								joint_trajectory_publisher_.publish(msg);
}

/// Called when a new JointState message is received.
void joint_state_callback(
								const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{

								current_joint_states_ = *joint_state_msg;

}

void track_transforms(tf::TransformBroadcaster br) {

								if (!(part_target_to_world.getOrigin().x() == 0 && part_target_to_world.getOrigin().y() == 0 && part_target_to_world.getOrigin().z() == 0)) {
																br.sendTransform(tf::StampedTransform(part_target_to_world, ros::Time::now(), "/world", "part_target"));
																br.sendTransform(tf::StampedTransform(gripper_target_to_part, ros::Time::now(), "/part_target", "gripper_target"));
								}

}


int main(int argc, char **argv)
{

								ros::init(argc, argv, "move_arm");
								ros:: NodeHandle n;
								tf::TransformListener listener;
								client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
								part_perception_client = n.serviceClient<part_perception::Part_Offset_Gripper>("/ariac/check_part_offset");
								belt_query_client = n.serviceClient<part_perception::Inventory_Predication>("/ariac/query_belt_part");

								group = new moveit::planning_interface::MoveGroupInterface("manipulator");
								group->startStateMonitor();
								ros::Subscriber gripper_sub = n.subscribe("ariac/gripper/state", 1000, gripper_callback);
								ros::Subscriber joint_state_subscriber = n.subscribe("/ariac/joint_states", 10, joint_state_callback);

								//transform boardcaster for tracking
								static tf::TransformBroadcaster br;


								//start_competition(n);

								group->setGoalTolerance(0.02);

								ros::ServiceServer service = n.advertiseService("/move_arm/toPose", add);

								joint_trajectory_publisher_ = n.advertise<trajectory_msgs::JointTrajectory>(
																"/ariac/arm/command", 10);

								ros::ServiceClient client_agv1 = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
								ros::ServiceClient client_agv2 = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
								osrf_gear::AGVControl Go_delivery;

								ros::AsyncSpinner spinner(4);
								spinner.start();


								ros::Rate loop_rate(100);
								while (ros::ok())
								{
																listener.waitForTransform("/world", "/vacuum_gripper_link", ros::Time(0), ros::Duration(10.0));
																listener.lookupTransform("/world", "/vacuum_gripper_link", ros::Time(0), gripper_transform);



																track_transforms(br);

																srv.request.enable = true;
																client.call(srv);
																ros::spinOnce();
																loop_rate.sleep();


								}
}



// geometry_msgs::Pose compute_offset_transform() {

//  geometry_msgs::Pose p;
//  p.position.x = 0;
//  p.position.y = 0;
//  p.position.z = 0;
//  p.orientation.x = 0;
//  p.orientation.y = 0;
//  p.orientation.z = 0;
//  p.orientation.w = 0;

//  ROS_INFO("Computing Offset");
//  if (!part_perception_client.exists()) {
//   ROS_INFO("Part Perception Service not Started");
//   part_perception_client.waitForExistence();
//  }

//  part_perception::Part_Offset_Gripper part_perception_srv;
//  part_perception_srv.request.check_part_offset = true;
//  part_perception_client.call(part_perception_srv);



//  if (!part_perception_srv.response.part_offset_info.transforms.empty()) {
//   float x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
//   float y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
//   float z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;
//   float camera_move_tolerance = 0.15;
//   if (std::abs(x) > camera_move_tolerance || std::abs(y) > camera_move_tolerance || std::abs(z) > camera_move_tolerance) {
//    ROS_INFO("Camera has moved!!!");

//    bool ok = false;
//    int i = 0;
//    while (!ok && i < 1000) {
//     part_perception_srv.request.check_part_offset = true;
//     part_perception_client.call(part_perception_srv);
//     x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
//     y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
//     z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;
//     if (std::abs(x) < camera_move_tolerance && std::abs(y) < camera_move_tolerance && std::abs(z) < camera_move_tolerance) {
//      ok = true;
//     }
//     ROS_INFO("Trying to perceive: %d", i);
//     i++;
//     sleep(0.3);
//    }

//   }
//   // ROS_INFO("Part Perception: %f",
//   //          part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x);
//   tf::Quaternion q(part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.x,
//                    part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.y,
//                    part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.z,
//                    part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.w);
//   tf::Matrix3x3 m(q);
//   double roll, pitch, yaw;
//   m.getRPY(roll, pitch, yaw);
//   ROS_INFO("Part Perception: x %f y %f z %f", part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x,
//            part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y,
//            part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z);
//   ROS_INFO("Part Perception: Roll %f Pitch %f Yaw %f", roll, pitch, yaw);

//   p.orientation.x = roll;
//   p.orientation.y = pitch;
//   p.orientation.z = yaw;

//   x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
//   y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
//   z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;


//   // p.position.x = - y * cos(yaw) + x * sin(yaw);
//   // p.position.y =  - y * sin(yaw) - x * cos(yaw);
//   // p.position.z = z * 2.0;


//   p.position.x = + y * sin(yaw)  - x * cos(yaw);
//   p.position.y =  y * cos(yaw) + x * sin(yaw);
//   p.position.z = z * 2.0;


//   ROS_INFO("offset ysin: %f", y * sin(yaw));
//   ROS_INFO("offset ycos: %f", y * cos(yaw));
//   ROS_INFO("offset xsin: %f", x * sin(yaw));
//   ROS_INFO("offset xcos: %f", x * cos(yaw));

//   //xsin xcos should be ok from top left







//  } else {
//   ROS_INFO("Gripper is holding nothing!");
//  }

//  return p;

// }
