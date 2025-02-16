#include <cstdlib>
#include <filesystem>
#include <string>

#include "MvCameraControl.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

#include "hik_camera/hik_camera_node.hpp"

namespace hik_camera {

HikCameraNode::HikCameraNode(rclcpp::NodeOptions options)
    : Node("hik_camera", options.use_intra_process_comms(true)) {
  RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

  initializeCamera();
  declareParameters();
  startCamera();

  params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &HikCameraNode::dynamicParametersCallback, this, std::placeholders::_1));

  capture_thread = std::thread(&HikCameraNode::captureLoop, this);
}

HikCameraNode::~HikCameraNode() {
  if (capture_thread.joinable()) {
    capture_thread.join();
  }
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
  }
  RCLCPP_INFO(this->get_logger(), "HikCamera Close!");

  if (recorder != nullptr) {
    recorder->stop();
    RCLCPP_INFO(this->get_logger(),
                "Recorder stopped! Video file %s has been saved",
                recorder->path_.string().c_str());
  }

  RCLCPP_INFO(this->get_logger(), "HikCameraNode Destructed!");
}

bool HikCameraNode::initializeCamera() {
  MV_CC_DEVICE_INFO_LIST device_list;

  // enum device
  while (rclcpp::ok()) {
    n_ret_ = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (n_ret_ != MV_OK) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to enumerate devices, retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else if (device_list.nDeviceNum == 0) {
      RCLCPP_ERROR(this->get_logger(), "No camera found, retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_INFO(this->get_logger(), "Found camera count = %d",
                  device_list.nDeviceNum);
      break;
    }
  }

  n_ret_ = MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
  if (n_ret_ != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create camera handle!");
    return false;
  }

  n_ret_ = MV_CC_OpenDevice(camera_handle_);
  if (n_ret_ != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera device!");
    return false;
  }

  // Get camera information
  n_ret_ = MV_CC_GetImageInfo(camera_handle_, &img_info_);
  if (n_ret_ != MV_OK) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get camera image info!");
    return false;
  }

  // Init convert param
  image_msg.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
  convert_param_.nWidth = img_info_.nWidthValue;
  convert_param_.nHeight = img_info_.nHeightValue;
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  return true;
}

void HikCameraNode::declareParameters() {
  MVCC_FLOATVALUE f_value;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;

  // Acquisition frame rate
  param_desc.description = "Acquisition frame rate in Hz";
  MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  acquisition_frame_rate =
      this->declare_parameter("acquisition_frame_rate", 165.0, param_desc);
  MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
  MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate",
                      acquisition_frame_rate);
  RCLCPP_INFO(this->get_logger(), "Acquisition frame rate: %f",
              acquisition_frame_rate);

  // Exposure time
  param_desc.description = "Exposure time in microseconds";
  MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
  RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

  // Gain
  param_desc.description = "Gain";
  MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  param_desc.integer_range[0].from_value = f_value.fMin;
  param_desc.integer_range[0].to_value = f_value.fMax;
  gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
  MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
  RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

  int status;

  // ADC Bit Depth
  param_desc.description = "ADC Bit Depth";
  param_desc.additional_constraints = "Supported values: Bits_8, Bits_12";
  adc_bit_depth =
      this->declare_parameter("adc_bit_depth", "Bits_8", param_desc);
  status = MV_CC_SetEnumValueByString(camera_handle_, "ADCBitDepth",
                                      adc_bit_depth.c_str());
  if (status == MV_OK) {
    RCLCPP_INFO(this->get_logger(), "ADC Bit Depth set to %s",
                adc_bit_depth.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set ADC Bit Depth, status = %d",
                 status);
  }

  // Pixel format
  param_desc.description = "Pixel Format";
  pixel_format =
      this->declare_parameter("pixel_format", "RGB8Packed", param_desc);
  status = MV_CC_SetEnumValueByString(camera_handle_, "PixelFormat",
                                      pixel_format.c_str());
  if (status == MV_OK) {
    RCLCPP_INFO(this->get_logger(), "Pixel Format set to %s",
                pixel_format.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Pixel Format, status = %d",
                 status);
  }

  // node params
  use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  camera_name = this->declare_parameter("camera_name", "camera");
  camera_info_url = this->declare_parameter(
      "camera_info_url", "package://hik_camera/config/camera_info.yaml");
  frame_id =
      this->declare_parameter("frame_id", camera_name + "_optical_frame");
  camera_topic =
      this->declare_parameter("camera_topic", camera_name + "/image");
  enable_recorder = this->declare_parameter("enable_recorder", false);
  save_path =
      this->declare_parameter("save_path", "./videos/");
}

void HikCameraNode::startCamera() {

  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                 : rmw_qos_profile_default;
  camera_pub =
      image_transport::create_camera_publisher(this, camera_topic, qos);

  MV_CC_StartGrabbing(camera_handle_);

  // Load camera info
  camera_info_manager =
      std::make_unique<camera_info_manager::CameraInfoManager>(this,
                                                               camera_name);
  if (camera_info_manager->validateURL(camera_info_url)) {
    camera_info_manager->loadCameraInfo(camera_info_url);
    camera_info_msg = camera_info_manager->getCameraInfo();
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                camera_info_url.c_str());
  }

  // Load Recorder
  if (enable_recorder) {
    recorder = std::make_unique<Recorder>(
        std::filesystem::path(save_path / std::string(std::to_string(std::time(nullptr)) + ".avi")),
        acquisition_frame_rate,
        cv::Size(img_info_.nWidthMax, img_info_.nHeightMax));
    recorder->start();
    RCLCPP_INFO(this->get_logger(), "Recorder started! Video file: %s",
                save_path.string().c_str());
  }
}

void HikCameraNode::captureLoop() {
  MV_FRAME_OUT out_frame;
  RCLCPP_INFO(this->get_logger(), "Publishing image!");

  image_msg.header.frame_id = frame_id;
  image_msg.encoding = "rgb8";

  while (rclcpp::ok()) {
    n_ret_ = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
    if (MV_OK == n_ret_) {
      convert_param_.pDstBuffer = image_msg.data.data();
      convert_param_.nDstBufferSize = image_msg.data.size();
      convert_param_.pSrcData = out_frame.pBufAddr;
      convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
      convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

      MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

      image_msg.header.stamp = this->now();
      image_msg.height = out_frame.stFrameInfo.nHeight;
      image_msg.width = out_frame.stFrameInfo.nWidth;
      image_msg.step = out_frame.stFrameInfo.nWidth * 3;
      image_msg.data.resize(image_msg.width * image_msg.height * 3);

      camera_info_msg.header = image_msg.header;
      camera_pub.publish(image_msg, camera_info_msg);

      // recorder
      if (recorder != nullptr) {
        recorder->addFrame(image_msg.data);
      }

      MV_CC_FreeImageBuffer(camera_handle_, &out_frame);

      static auto last_log_time = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time)
              .count() >= 3) {
        MVCC_FLOATVALUE f_value;
        MV_CC_GetFloatValue(camera_handle_, "ResultingFrameRate", &f_value);
        RCLCPP_DEBUG(this->get_logger(), "ResultingFrameRate: %f Hz",
                     f_value.fCurValue);
        last_log_time = now;
      }

    } else {
      RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", n_ret_);
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_StartGrabbing(camera_handle_);
      fail_count++;
    }

    if (fail_count > 5) {
      RCLCPP_FATAL(this->get_logger(), "Camera failed!");
      rclcpp::shutdown();
    }
  }
}

rcl_interfaces::msg::SetParametersResult
HikCameraNode::dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    const auto &type = param.get_type();
    const auto &name = param.get_name();
    int status = MV_OK;

    if (type == rclcpp::ParameterType::PARAMETER_DOUBLE) {
      if (name == "gain") {
        status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + name;
        continue;
      }
    } else if (type == rclcpp::ParameterType::PARAMETER_INTEGER) {
      if (name == "exposure_time") {
        status =
            MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + name;
        continue;
      }
    } else {
      result.successful = false;
      result.reason = "Unsupported parameter type for: " + name;
      continue;
    }

    if (status != MV_OK) {
      result.successful = false;
      result.reason =
          "Failed to set " + name + ", status = " + std::to_string(status);
    }
  }

  return result;
}
} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)