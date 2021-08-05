#include "capstone/path_manager.h"

#include <nav_msgs/OccupancyGrid.h>

namespace capstone
{
PathManager::PathManager(ros::NodeHandle &nh)
  : nh_(nh)
  , is_done_sub_(nh_.subscribe("/is_done", 1, &PathManager::isDoneCb, this))
  , robot_pose_sub_(nh_.subscribe(ROBOT_POSE_TOPIC, 1, &PathManager::robotPoseCb, this))
{
  robot_path_points_.emplace_back(robot_pose_point_);
  ROS_INFO_STREAM("Ready to save in an image the path you will command the robot to do");
}

PathManager::~PathManager()
{
  lock_.unlock();
  robot_pose_sub_.shutdown();
  nh_.shutdown();
}

void PathManager::isDoneCb(const std_msgs::BoolConstPtr &msg)
{
  if (msg->data)
  {
    std::time_t date_time;
    std::tm *date_time_d;
    std::time(&date_time);
    date_time_d = std::localtime(&date_time);
    int char_size = 80;
    char current_date_time[char_size];
    std::strftime(current_date_time, char_size, "%Y-%m-%dT%H:%M:%S", date_time_d);

    std::string img_path = std::string(TMP_PAHT) + "capstone_" +
                           std::string(current_date_time) + std::string(PNG_FILE_EXTENSION);

    cv::Mat img(HEIGHT, WIDTH, CV_8UC3, WHITE);
    cv::Point cv_prev_point =
        rosWorld2cvPixel(robot_path_points_.front().x, robot_path_points_.front().y);
    cv::circle(img, cv_prev_point, RADIUS, BLUE);
    cv::Point cv_point;
    cv::Point x_axis_start(WIDTH / 2, HEIGHT / 2);
    cv::Point x_axis_end(WIDTH, HEIGHT / 2);
    cv::line(img, x_axis_start, x_axis_end, RED);

    cv::Point y_axis_start(WIDTH / 2, HEIGHT / 2);
    cv::Point y_axis_end(WIDTH / 2, 0);
    cv::line(img, y_axis_start, y_axis_end, GREEN);

    for (unsigned int i = 1; i < robot_path_points_.size(); ++i)
    {
      cv_point = rosWorld2cvPixel(robot_path_points_.at(i).x, robot_path_points_.at(i).y);
      cv::line(img, cv_prev_point, cv_point, BLACK);
      cv_prev_point = cv_point;
    }
    cv::circle(img, cv_prev_point, RADIUS, DARK_GREEN);

    cv::rotate(img, img, cv::ROTATE_90_CLOCKWISE);

    cv::Point circle_cv_point(CIRCLE_LEGEND_POS, CIRCLE_LEGEND_POS);
    cv::circle(img, circle_cv_point, RADIUS, BLUE);
    cv::Point text_cv_point = circle_cv_point;
    text_cv_point.x += LEGEND_PADDING_X;
    text_cv_point.y += LEGEND_PADDING_Y;
    cv::putText(img, "START", text_cv_point, cv::FONT_HERSHEY_DUPLEX, FONT_SCALE, BLUE);

    circle_cv_point.y += 10;
    cv::circle(img, circle_cv_point, RADIUS, DARK_GREEN);
    text_cv_point = circle_cv_point;
    text_cv_point.x += LEGEND_PADDING_X;
    text_cv_point.y += LEGEND_PADDING_Y;
    cv::putText(img, "END", text_cv_point, cv::FONT_HERSHEY_DUPLEX, FONT_SCALE, DARK_GREEN);

    text_cv_point = cv::Point(WIDTH / 2, HEIGHT);
    text_cv_point.x += AXIS_PADDING;
    text_cv_point.y -= AXIS_PADDING;
    cv::putText(img, "X-axis", text_cv_point, cv::FONT_HERSHEY_DUPLEX, FONT_SCALE, RED);

    text_cv_point = x_axis_end;
    text_cv_point.x -= Y_AXIS_X_PADDING;
    text_cv_point.y -= AXIS_PADDING;
    cv::putText(img, "Y-axis", text_cv_point, cv::FONT_HERSHEY_DUPLEX, FONT_SCALE, GREEN);

    cv::imshow("Robot's path", img);
    cv::waitKey(0);

    ROS_INFO_STREAM("Saving the robot's path img in " << img_path);
    if (!cv::imwrite(img_path, img))
    {
      ROS_ERROR_STREAM("Failed to save the robot's path img");
    }
  }
  robot_path_points_.clear();
  robot_path_points_.emplace_back(robot_pose_point_);
}

void PathManager::robotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
{
  boost::recursive_mutex::scoped_lock lock(lock_);
  robot_pose_point_ = msg->pose.pose.position;

  if (robot_path_points_.back() != robot_pose_point_)
  {
    robot_path_points_.emplace_back(robot_pose_point_);
  }
}

cv::Point PathManager::rosWorld2cvPixel(double ros_wrold_x, double ros_world_y) const
{
  double grid_x = static_cast<int>(floor((ros_wrold_x + ORIGIN_X) / RESOLUTION));
  double grid_y = static_cast<int>(floor((ros_world_y + ORIGIN_Y) / RESOLUTION));
  return cv::Point(grid_x, HEIGHT - grid_y);
}
}  // namespace capstone
