#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"

// 생성자. 위의 .h 헤더파일에서 정의한 class에 대한 생성자.
// 위 클래스가 처음 생성될 때, 진행되는 Initialize 작업들

// initsubscriber에 추가로 ik로 푼 joint 값 전달해 줄 것임.
OpenManipulatorTeleop::OpenManipulatorTeleop()      
: node_handle_(""),
  priv_node_handle_("~")
{
  /************************************************************
  ** Initialize variables
  ************************************************************/
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  /************************************************************
  ** Initialize ROS Subscribers and Clients
  ************************************************************/
  initSubscriber();
  initClient();

  disableWaitingForEnter();
  ROS_INFO("OpenManipulator teleoperation using keyboard start");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  restoreTerminalSettings();
  ROS_INFO("Terminate OpenManipulator Joystick");
  ros::shutdown();
}

void OpenManipulatorTeleop::initClient()
{
     printf("IN initClient\n");
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}

void OpenManipulatorTeleop::initSubscriber()
{  
    printf("IN initSubscriber\n");
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  // 추가됨.
  // callback 함수 만들어야함.
  ik_pos_jwK_sub_ = node_handle_.subscribe("ik_position", 10, &OpenManipulatorTeleop::setGoal, this);

}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for (std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}
// 추가됨.??
// void

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText()
{
  {
//   printf("\n");
//   printf("---------------------------\n");
//   printf("Control Your OpenManipulator!\n");
//   printf("---------------------------\n");
//   printf("w : increase x axis in task space\n");
//   printf("s : decrease x axis in task space\n");
//   printf("a : increase y axis in task space\n");
//   printf("d : decrease y axis in task space\n");
//   printf("z : increase z axis in task space\n");
//   printf("x : decrease z axis in task space\n");
//   printf("\n");
//   printf("y : increase joint 1 angle\n");
//   printf("h : decrease joint 1 angle\n");
//   printf("u : increase joint 2 angle\n");
//   printf("j : decrease joint 2 angle\n");
//   printf("i : increase joint 3 angle\n");
//   printf("k : decrease joint 3 angle\n");
//   printf("o : increase joint 4 angle\n");
//   printf("l : decrease joint 4 angle\n");
//   printf("\n");
//   printf("g : gripper open\n");
//   printf("f : gripper close\n");
//   printf("       \n");
//   printf("1 : init pose\n");
//   printf("2 : home pose\n");
//   printf("       \n");
//   printf("q to quit\n");
//   printf("---------------------------\n");
  }
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("---------------------------\n");

}

void OpenManipulatorTeleop::setGoal(const std_msgs::Float32MultiArray::ConstPtr &pos_array)
{
    printf("Welcome setGoal!\n");
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(NUM_OF_JOINT, 0.0);

  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = 2.0;

  // 추가됨.
  printf("Subscribe success!\n");
    printf("Subscribe success!\n");
  joint_name.push_back("joint1"); joint_angle.push_back(pos_array->data[0]);
  joint_name.push_back("joint2"); joint_angle.push_back(pos_array->data[1]);
  joint_name.push_back("joint3"); joint_angle.push_back(pos_array->data[2]);
  joint_name.push_back("joint4"); joint_angle.push_back(pos_array->data[3]);
  setJointSpacePath(joint_name, joint_angle, path_time);
}

void OpenManipulatorTeleop::restoreTerminalSettings(void)
{
  tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_teleop_keyboard");
  OpenManipulatorTeleop openManipulatorTeleop;

  char ch;
  openManipulatorTeleop.printText();

  /*
  원래 구조:
  ros::spinOnce()를 사용했으니까 한번 subscribe온거 확인하고, 
  다음 작업함.(printText함수가서 이것저것 프린트)
  원래 subscribe했던거 : 위에서 정의된 subscribe 내용들
  (현재 로봇의 joint_status, kinematics_pos)

  여기에 
  
  */ 
 ros::spin();

//   while (ros::ok())
//   {
//     ros::spinOnce();                    // 한번 돌고
//     openManipulatorTeleop.printText();
//     // ros::spinOnce();
//     // openManipulatorTeleop.setGoal(ch);
//   }

  return 0;
}
