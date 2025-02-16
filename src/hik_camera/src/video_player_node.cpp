#include "hik_camera/video_player_node.hpp"
#include <cstring>
#include <filesystem>
#include <image_transport/image_transport.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace hik_camera {

VideoPlayerNode::VideoPlayerNode(const rclcpp::NodeOptions &options)
    : Node("video_player"), frame_cnt(0) {
  RCLCPP_INFO(this->get_logger(), "VideoPlayerNode is running!");

  // get parameters
  video_path = this->declare_parameter<std::string>("video_path", "/home/rm/RCS2025-log/video/1737586785.avi");
  camera_info_url = this->declare_parameter<std::string>(
      "camera_info_url", "package://hik_camera/config/camera_info.yaml");
  camera_name = this->declare_parameter("camera_name", "camera");
  frame_id =
      this->declare_parameter<std::string>("frame_id", "camera_optical_frame");
  frame_rate = this->declare_parameter<int>("frame_rate", 200);
  start_frame = this->declare_parameter<int>("start_frame", 0);
  keep_looping = this->declare_parameter<bool>("keep_looping", true);

  std::filesystem::path video_file(video_path);

  if (!std::filesystem::exists(video_file)) {
    RCLCPP_ERROR(this->get_logger(), "Video file %s does not exist!",
                 video_path.c_str());
    rclcpp::shutdown();
    return;
  }

  cap.open(video_path);
  if (!cap.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open video file %s!",
                 video_path.c_str());
    rclcpp::shutdown();
    return;
  }

  // set image msg
  image_msg = std::make_shared<sensor_msgs::msg::Image>();
  image_msg->header.frame_id = frame_id;
  image_msg->encoding = sensor_msgs::image_encodings::BGR8;
  image_msg->width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  image_msg->height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  image_msg->step = image_msg->width * 3;
  image_msg->data.resize(image_msg->height * image_msg->step);

  // set camera info
  camera_info_manager =
      std::make_shared<camera_info_manager::CameraInfoManager>(this,
                                                               camera_name);
  if (camera_info_manager->validateURL(camera_info_url)) {
    camera_info_manager->loadCameraInfo(camera_info_url);
    camera_info = camera_info_manager->getCameraInfo();
  } else {
    camera_info_manager->setCameraName(video_path);
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.header.frame_id = "camera_optical_frame";
    camera_info.header.stamp = this->now();
    camera_info.width = image_msg->width;
    camera_info.height = image_msg->height;
    camera_info_manager->setCameraInfo(camera_info);
    RCLCPP_WARN(this->get_logger(),
                "Failed to load camera info from %s, use default camera info!",
                camera_info_url.c_str());
  }
  camera_info.header.frame_id = frame_id;
  camera_info.header.stamp = this->now();

  // pub
  camera_pub = image_transport::create_camera_publisher(this, "image_raw");

  // loop
  loop_rate = std::make_shared<rclcpp::WallRate>(frame_rate);
  timer = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
    cap >> frame;
    if (frame.empty()) {
      RCLCPP_INFO(this->get_logger(), "Video end!");
      if (!keep_looping) {
        rclcpp::shutdown();
        return;
      } else {
        cap.open(video_path);
        frame_cnt = 0;
        return;
      }
    }
    memcpy(image_msg->data.data(), frame.data,
           image_msg->step * image_msg->height);
    frame_cnt++;
    if (frame_cnt < start_frame) {
      RCLCPP_INFO(this->get_logger(), "Skip frame %d", frame_cnt);
      return;
    }
    image_msg->header.stamp = camera_info.header.stamp = this->now();
    camera_pub.publish(*image_msg, camera_info);
    loop_rate->sleep();
  });

  rclcpp::spin(this->get_node_base_interface());
}
} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::VideoPlayerNode)