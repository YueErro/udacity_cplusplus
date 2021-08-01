#include "capstone/robot_pose_publisher.h"

namespace capstone
{
RobotPosePublisher::RobotPosePublisher(ros::NodeHandle& nh)
  : nh_(nh)
  , cmd_vel_sub_(nh_.subscribe("/cmd_vel", 1, &RobotPosePublisher::cmdVelCb, this))
  , robot_pose_pub_(nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(ROBOT_POSE_TOPIC, 1))
  , rate_(50)
  , odom_frame_("odom")
  , robot_frame_("base_footprint")
  , listener_(tf_buffer_)
{
}

RobotPosePublisher::~RobotPosePublisher()
{
  tf_buffer_.clear();
  lock_.unlock();
  nh_.shutdown();
}

void RobotPosePublisher::cmdVelCb(const geometry_msgs::TwistConstPtr& msg)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  if (msg->linear.x != 0.0 || msg->angular.z != 0.0)
  {
    try
    {
      odom_to_base_ = tf_buffer_.lookupTransform(odom_frame_, robot_frame_,
                                                 ros::Time::now(), ros::Duration(0.1));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM(ex.what());
      ros::Duration(0.1).sleep();
    }

    tf2::fromMsg(odom_to_base_.transform, odom_to_base_tf_);
    tf2::toMsg(odom_to_base_tf_, robot_pose_msg_.pose.pose);

    robot_pose_msg_.header.frame_id = odom_frame_;
    robot_pose_msg_.header.stamp = ros::Time::now();

    robot_pose_pub_.publish(robot_pose_msg_);

    rate_.sleep();
  }
}
}  // namespace capstone
