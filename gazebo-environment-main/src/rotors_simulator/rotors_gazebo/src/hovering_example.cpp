/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

ros::Publisher trajectory_pub;

void goToPos(Eigen::Vector3d desired_position, ros::NodeHandle nh){
  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  double desired_yaw = 0.0;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);
  ros::Duration(0.1).sleep();
  // ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
  //          nh.getNamespace().c_str(),
  //          desired_position.x(),
  //          desired_position.y(),
  //          desired_position.z());
  trajectory_pub.publish(trajectory_msg);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "hovering_example");
  ros::NodeHandle nh;
  trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  }
  else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 8 seconds to let the Gazebo GUI show up.
  ros::Duration(8.0).sleep();

  /* init vins*/
  Eigen::Vector3d desired_position;
  int num = 20;
  double step = 0.05;
  // up
  for (int i=0; i<num; i++){
    desired_position = Eigen::Vector3d(0.0, 0.0, num*step);
    goToPos(desired_position, nh);
  }
  // left
  for (int i=0; i<num; i++){
    desired_position = Eigen::Vector3d(0.0, num*step, 1.0);
    goToPos(desired_position, nh);
  }
  //right
  for (int i=0; i<num; i++){
    desired_position = Eigen::Vector3d(0.0, 1.0 - 2*num*step, 1.0);
    goToPos(desired_position, nh);
  }
  // back to center
  for (int i=0; i<num; i++){
    desired_position = Eigen::Vector3d(0.0, num*step - 1.0, 1.0);
    goToPos(desired_position, nh);
  }
  // forward
  for (int i=0; i<num; i++){
    desired_position = Eigen::Vector3d(num*step, 0.0, 1.0);
    goToPos(desired_position, nh);
  }
  // back to center
  for (int i=0; i<num; i++){
    desired_position = Eigen::Vector3d(1.0 - num*step, 0.0, 1.0);
    goToPos(desired_position, nh);
  }

  ros::spin();
}
