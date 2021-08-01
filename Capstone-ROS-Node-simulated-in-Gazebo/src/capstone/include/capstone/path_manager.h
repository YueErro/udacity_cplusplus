#ifndef PATH_MANAGER
#define PATH_MANAGER

#include "capstone/robot_pose_publisher.h"

#include <std_msgs/Bool.h>
#include <nav_msgs/MapMetaData.h>

#include <opencv2/opencv.hpp>

#define FONT_SCALE 0.25
#define RADIUS 3.0
#define RESOLUTION 0.025
#define WIDTH 720
#define HEIGHT 720
#define ORIGIN_X 9  // WIDTH * RESOLUTION
#define ORIGIN_Y 9  // HEIGHT * RESOLUTION
#define CIRCLE_LEGEND_POS 10
#define LEGEND_PADDING_X 5
#define LEGEND_PADDING_Y 3
#define AXIS_PADDING 2
#define Y_AXIS_X_PADDING 30

namespace capstone
{
static constexpr char TMP_PAHT[] = "/tmp/";
static constexpr char PNG_FILE_EXTENSION[] = ".png";

static const cv::Scalar BLUE(cv::Scalar(255, 0, 0));
static const cv::Scalar GREEN(cv::Scalar(0, 255, 0));
static const cv::Scalar DARK_GREEN(cv::Scalar(0, 127, 0));
static const cv::Scalar RED(cv::Scalar(0, 0, 255));
static const cv::Scalar BLACK(cv::Scalar(0, 0, 0));
static const cv::Scalar WHITE(cv::Scalar(255, 255, 255));

class PathManager
{
public:
  PathManager(ros::NodeHandle &nh);
  ~PathManager();

protected:
  ros::NodeHandle nh_;
  ros::Subscriber is_done_sub_;
  ros::Subscriber robot_pose_sub_;

  geometry_msgs::Point robot_pose_point_;
  std::vector<geometry_msgs::Point> robot_path_points_;

  boost::recursive_mutex lock_;

  void isDoneCb(const std_msgs::BoolConstPtr &msg);
  void robotPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

private:
  cv::Point rosWorld2cvPixel(double ros_wrold_x, double ros_world_y) const;
};
}  // namespace capstone

#endif  // PATH_MANAGER
