﻿#ifndef OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
#define OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_

#include <termios.h>
#include <vector> // 추가됨.

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

#define NUM_OF_JOINT 4
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

class OpenManipulatorTeleop
{
public:
  OpenManipulatorTeleop();
  ~OpenManipulatorTeleop();

  // update
  void printText();
  // void setGoal(char ch);

private:
  /*****************************************************************************
  ** ROS NodeHandle
  *****************************************************************************/
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  /*****************************************************************************
  ** Variables
  *****************************************************************************/
  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;

  /*****************************************************************************
  ** Init Functions
  *****************************************************************************/
  void initSubscriber();
  void initClient();

  /*****************************************************************************
  ** ROS Subscribers, Callback Functions and Relevant Functions
  *****************************************************************************/
  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;
  // 추가됨.
  ros::Subscriber ik_pos_jwK_sub_;

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);
  void setGoal(const std_msgs::Float32MultiArray::ConstPtr &pos_array);

  /*****************************************************************************
  ** ROS Clients and Callback Functions
  *****************************************************************************/
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_tool_control_client_;

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);
  bool setToolControl(std::vector<double> joint_angle);

  /*****************************************************************************
  ** Others
  *****************************************************************************/
  struct termios oldt_;

  void disableWaitingForEnter(void);
  void restoreTerminalSettings(void);
  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();
};

#endif // OPEN_MANIPULATOR_TELEOP_KEYBOARD_H_
