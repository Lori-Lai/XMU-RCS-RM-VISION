#ifndef HIK_CAMERA__HIK_CAMERA_NODE_HPP_
#define HIK_CAMERA__HIK_CAMERA_NODE_HPP_

#include <image_transport/camera_publisher.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <string>

#include "CameraParams.h"
#include "MvErrorDefine.h"

#include "camera_info_manager/camera_info_manager.hpp"
#include "hik_camera/recorder.hpp"

namespace hik_camera {
class HikCameraNode : public rclcpp::Node {
public:
  explicit HikCameraNode(rclcpp::NodeOptions options);
  ~HikCameraNode() override;

private:
  bool initializeCamera();
  void declareParameters();
  void startCamera();
  void captureLoop();

  // Param set callback
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(const std::vector<rclcpp::Parameter> &parameters);
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      params_callback_handle_;

  // HIK SDK API
  void *camera_handle_ = nullptr;
  int n_ret_ = MV_OK;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  rcl_interfaces::msg::ParameterDescriptor param_desc;

  // camera param
  std::basic_string<char> camera_info_url;
  double acquisition_frame_rate;
  double exposure_time;
  double gain;
  std::string adc_bit_depth;
  std::string pixel_format;

  // node param
  bool use_sensor_data_qos;
  std::string camera_name;
  std::string frame_id;
  std::string camera_topic;

  // Recorder
  bool enable_recorder;
  std::unique_ptr<Recorder> recorder;
  std::filesystem::path save_path;

  sensor_msgs::msg::Image image_msg;
  sensor_msgs::msg::CameraInfo camera_info_msg;
  image_transport::CameraPublisher camera_pub;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;

  std::thread capture_thread;
  int fail_count = 0;
};
} // namespace hik_camera

#endif