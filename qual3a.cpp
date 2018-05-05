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
#include "MyCompetitionClass.h"

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
                "/ariac/proximity_sensor_1", 10, &MyCompetitionClass::proximity_sensor_callback, &comp_class);
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
                "/ariac/laser_profiler_1", 10, &MyCompetitionClass::laser_profiler_callback, &comp_class);
        tf::TransformListener listener;
        //ROS_INFO("Setup complete.");
        //start_competition(node);
        ros::ServiceClient client = node.serviceClient<move_arm::Pick>("/move_arm/toPose");
        move_arm::Pick Movesrv;
        std::vector<std::vector<float> > bin = {{-0.2,0.33,0.745},{-0.4,0.13,0.724},{-0.1,-0.335,0.745},{-0.233,-0.468,0.745},{-0.36,-0.335,0.745},{0.4,3.15,0.75},{-0.4,0.33,0.75},{-0.2,0.13,0.75},{-0.5,-0.46,0.75},{-0.5,-0.6,0.75},{-0.1,-0.6,0.75},{-0.07,0.13,0.75}};
        while(ros::ok()) {
                if(!comp_class.kit_priority_queue.empty()) {
                        int num_of_kits = comp_class.kit_priority_queue.size();
                        ROS_INFO_STREAM("there are  " << comp_class.kit_priority_queue.size()<<" kits\n" );
                        comp_class.kit_priority_queue.erase(num_of_kits);
                        ROS_INFO_STREAM("there are  " << comp_class.kit_priority_queue.size()<<" kits\n" );
                        //    std::vector<geometry_msgs::Pose> world_pose_list;
                        //
                        //
                        //    Access to every order(objects) in one specific order and reassign the pose relative to world corrdinate
                        for(int i = 0; i< comp_class.kit_priority_queue[num_of_kits].order_in_kit_list.size(); i++) {
                                ROS_INFO_STREAM("Kit has "<< comp_class.kit_priority_queue[num_of_kits].order_in_kit_list.size()<< " of objects");
                                //ROS_INFO_STREAM("Object type is "<< comp_class.kit_priority_queue[i].first);
                                //      geometry_msgs::Pose world_pose = comp_class.desired_part_pose(listener,i);
                                //  if (world_pose.position.x != 0)
                                //      world_pose_list.push_back(world_pose);
                        }
                        /*
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

                           } */
                }

                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
