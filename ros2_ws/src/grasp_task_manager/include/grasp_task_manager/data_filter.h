#ifndef DATA_FILTER_H
#define DATA_FILTER_H
#include "grasp_common.h"

class DataFilter {
 public:
  // 
  DataFilter(size_t window_size) : window_size_(window_size) {}

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_window_.clear();
  }

  // 
  void applyMeanFilter(const grasp_pose_t& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    // 
    pose_window_.push_back(pose);

    // ,
    if (pose_window_.size() > window_size_) {
      pose_window_.erase(pose_window_.begin());
    }
  }

  // 
  double getPositionStdDev(grasp_pose_t& mean_pos, size_t& size) {
    std::lock_guard<std::mutex> lock(mutex_);
    size = pose_window_.size();
    mean_pos = getMeanPose();
    double sum_squared_diff = 0.0;
    for (const auto& pos : pose_window_) {
      sum_squared_diff += (pos.center_pos - mean_pos.center_pos).length2();
    }
    return pose_window_.empty()
               ? 0.0
               : std::sqrt(sum_squared_diff / pose_window_.size());
  }

 private:
  // 
  grasp_pose_t getMeanPose() {
    grasp_pose_t mean_pos;
    mean_pos.center_pos = tf2::Vector3(0.0, 0.0, 0.0);
    mean_pos.left_pos = mean_pos.center_pos;
    mean_pos.right_pos = mean_pos.center_pos;
    mean_pos.left_press_pos = mean_pos.center_pos;
    mean_pos.right_press_pos = mean_pos.center_pos;
    mean_pos.roll = 0.0;
    mean_pos.pitch = 0.0;
    mean_pos.yaw = 0.0;
    mean_pos.width = 0.0;
    for (const auto& pos : pose_window_) {
      mean_pos.center_pos += pos.center_pos;
      mean_pos.left_pos += pos.left_pos;
      mean_pos.right_pos += pos.right_pos;
      mean_pos.left_press_pos += pos.left_press_pos;
      mean_pos.right_press_pos += pos.right_press_pos;

      mean_pos.roll += pos.roll;
      mean_pos.pitch += pos.pitch;
      mean_pos.yaw += pos.yaw;
      mean_pos.width += pos.width;
    }
    size_t size = pose_window_.size();
    if (!pose_window_.empty()) {
      mean_pos.grasp_type = pose_window_.back().grasp_type;
      mean_pos.center_pos = mean_pos.center_pos / size;
      mean_pos.left_pos = mean_pos.left_pos / size;
      mean_pos.right_pos = mean_pos.right_pos / size;
      mean_pos.left_press_pos = mean_pos.left_press_pos / size;
      mean_pos.right_press_pos = mean_pos.right_press_pos / size;

      mean_pos.roll /= size;
      mean_pos.pitch /= size;
      mean_pos.yaw /= size;
      mean_pos.width /= size;
    }

    return mean_pos;
  }

 protected:
  std::mutex mutex_;
  size_t window_size_;
  std::vector<grasp_pose_t> pose_window_;
};

#endif
