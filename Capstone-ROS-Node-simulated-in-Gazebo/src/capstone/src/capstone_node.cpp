#include "capstone/path_manager.h"

#include <ros/callback_queue.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "capstone_node");
  ros::NodeHandle nh("~");

  ros::NodeHandle path_manager_nh("~");
  ros::CallbackQueue path_manager_queue;
  path_manager_nh.setCallbackQueue(&path_manager_queue);

  ros::NodeHandle robot_pose_pub_nh("");
  ros::CallbackQueue robot_pose_pub_queue;
  robot_pose_pub_nh.setCallbackQueue(&robot_pose_pub_queue);

  capstone::PathManager path_manager(path_manager_nh);
  capstone::RobotPosePublisher robot_pose_publisher(robot_pose_pub_nh);

  ros::AsyncSpinner path_manager_spinner(1, &path_manager_queue);
  ros::AsyncSpinner robot_pose_pub_spinner(1, &robot_pose_pub_queue);
  path_manager_spinner.start();
  robot_pose_pub_spinner.start();

  while (ros::ok())
  {
    ros::Duration(0.5).sleep();
  }

  return EXIT_SUCCESS;
}
