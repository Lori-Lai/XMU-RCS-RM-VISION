#ifndef HIK_CAMERA__RECORDER_HPP_
#define HIK_CAMERA__RECORDER_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <opencv2/videoio.hpp>
#include <thread>

namespace hik_camera {
class Recorder {
public:
  using Frame = std::vector<unsigned char>;
  Recorder(const std::filesystem::path &file, int fps, cv::Size size);
  ~Recorder();

  void addFrame(const Frame &frame);
  void start();
  void stop();

  std::filesystem::path path_;

private:
  void recorderThread();

  cv::Size size_;
  int fps_;
  cv::VideoWriter writer_;

  std::deque<Frame> frames_queue;

  std::mutex mutex_;
  std::atomic<bool> recording_;
  std::condition_variable cv_;
  std::thread recorder_thread_;
};
} // namespace hik_camera

#endif // HIK_CAMERA__RECORDER_HPP_