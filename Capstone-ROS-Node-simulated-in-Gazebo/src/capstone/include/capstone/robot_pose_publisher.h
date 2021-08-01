#ifndef ROBOT_POSE_PUBLISHER
#define ROBOT_POSE_PUBLISHER

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <boost/thread.hpp>

namespace capstone
{
static constexpr char ROBOT_POSE_TOPIC[] = "/robot_pose";

class RobotPosePublisher
{
public:
  RobotPosePublisher(ros::NodeHandle &nh);
  ~RobotPosePublisher();

protected:
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Publisher robot_pose_pub_;
  ros::Rate rate_;

  std::string odom_frame_;
  std::string robot_frame_;

  boost::recursive_mutex lock_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener listener_;

  geometry_msgs::PoseWithCovarianceStamped robot_pose_msg_;
  geometry_msgs::TransformStamped odom_to_base_;
  tf2::Transform odom_to_base_tf_;

  void cmdVelCb(const geometry_msgs::TwistConstPtr &msg);
};
}  // namespace capstone

#endif  // ROBOT_POSE_PUBLISHER
