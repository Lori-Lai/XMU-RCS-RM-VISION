#ifndef HIK_CAMERA__VIDEO_PLAYER_NODE_
#define HIK_CAMERA__VIDEO_PLAYER_NODE_

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <memory>
#include <rclcpp/rate.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>

#include <opencv2/opencv.hpp>

namespace hik_camera {
class VideoPlayerNode : public rclcpp::Node {
public:
  explicit VideoPlayerNode(const rclcpp::NodeOptions &options);

private:
  std::string video_path;
  std::string camera_name;
  std::string camera_info_url;
  std::string frame_id;
  int frame_rate;
  image_transport::CameraPublisher camera_pub;
  bool keep_looping;
  cv::VideoCapture cap;
  cv::Mat frame;
  sensor_msgs::msg::Image::SharedPtr image_msg;
  sensor_msgs::msg::CameraInfo camera_info;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
  rclcpp::TimerBase ::SharedPtr timer;
  rclcpp::WallRate::SharedPtr loop_rate;
  int start_frame;
  int frame_cnt;
};
} // namespace hik_camera

#endif // HIK_CAMERA__VIDEO_PLAYER_NODE_