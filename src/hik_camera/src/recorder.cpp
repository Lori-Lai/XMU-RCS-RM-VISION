#include "hik_camera/recorder.hpp"
#include <mutex>
#include <opencv2/imgproc.hpp>

namespace hik_camera {

    Recorder::Recorder(const std::filesystem::path &file, int fps, cv::Size size) : path_(file), size_(size), fps_(fps) {
    }

    void Recorder::start() {
        if(!std::filesystem::exists(path_)){
            std::filesystem::create_directories(path_.parent_path());
        }

        writer_.open(path_.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, size_, true);
        recording_ = true;
        recorder_thread_ = std::thread(&Recorder::recorderThread, this);
    }

    Recorder::~Recorder() {
        stop();
    }

    void Recorder::addFrame(const Frame &frame){
        mutex_.lock();
        if(frames_queue.size() < 5){
            frames_queue.push_back(frame);
        }
        mutex_.unlock();
        cv_.notify_one();
    }

    void Recorder::stop() {
        recording_ = false;
        cv_.notify_all();
        recorder_thread_.join();
    }

    void Recorder::recorderThread() {
        while (recording_) {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !frames_queue.empty() || !recording_; });
            if(!recording_){
                break;
            }
            auto buffer = std::move(frames_queue.front());
            frames_queue.pop_front();
            lock.unlock();
            if(!buffer.empty()){
                cv::Mat frame(size_, CV_8UC3, buffer.data());
                cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
                writer_.write(frame);
            }
        }
    }

} // namespace hik_camera