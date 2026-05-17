#ifndef DETECT_ARUCO_MARKERS__ACTION_HPP
#define DETECT_ARUCO_MARKERS__ACTION_HPP
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "bt_subscriber_node.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/aruco.hpp"

using namespace behavior_tree;

class GetImageNode : public BtSubscriberNode<sensor_msgs::msg::Image> {
 public:
  GetImageNode(const std::string &xml_tag_name, const std::string &topic_name,
               const BT::NodeConfiguration &conf)
      : BtSubscriberNode<sensor_msgs::msg::Image>(xml_tag_name, topic_name,
                                                  conf) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<sensor_msgs::msg::Image>("image"),
        BT::InputPort<sensor_msgs::msg::CameraInfo>("camera_info"),
        BT::InputPort<std::string>("parameters"),
        BT::OutputPort<std::vector<std::tuple<int, geometry_msgs::msg::Pose>>>(
            "detections")};
  }
  // return providedBasicPorts({
  //     BT::OutputPort<sensor_msgs::msg::Image>("message_out",
  //                                             "Current Camera Image"),
  // });
  // }

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus on_tick() override {
    // Retrieve inputs
    auto image_msg = getInput<sensor_msgs::msg::Image>("image");
    auto camera_info_msg =
        getInput<sensor_msgs::msg::CameraInfo>("camera_info");
    auto parameters = getInput<std::string>("parameters");

    if (!image_msg || !camera_info_msg || !parameters) {
      RCLCPP_ERROR(node_->get_logger(), "Missing input ports");
      return BT::NodeStatus::FAILURE;
    }

    // Convert image message to OpenCV Mat
    cv::Mat image;
    try {
      auto cv_image = cv_bridge::toCvShare(*image_msg, "bgr8");
      image = cv_image->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
      return BT::NodeStatus::FAILURE;
    }

    // Parse parameters for ArUco
    auto dictionary_id = cv::aruco::DICT_6X6_50;  // Default dictionary
    if (parameters.value() == "DICT_4X4_50") {
      dictionary_id = cv::aruco::DICT_4X4_50;
    }

    auto dictionary = cv::aruco::getPredefinedDictionary(dictionary_id);
    auto parameters_ptr = cv::aruco::DetectorParameters::create();

    // Detect ArUco markers
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids,
                             parameters_ptr);

    if (marker_ids.empty()) {
      RCLCPP_INFO(node_->get_logger(), "No ArUco markers detected");
      return BT::NodeStatus::FAILURE;
    }

    // Compute pose for each marker
    std::vector<std::tuple<int, geometry_msgs::msg::Pose>> detections;
    if (camera_info_msg) {
      cv::Mat camera_matrix(3, 3, CV_64F,
                            const_cast<double *>(camera_info_msg->k.data()));
      cv::Mat dist_coeffs(1, 5, CV_64F,
                          const_cast<double *>(camera_info_msg->d.data()));

      for (size_t i = 0; i < marker_ids.size(); ++i) {
        cv::Vec3d rvec, tvec;
        cv::aruco::estimatePoseSingleMarkers(
            marker_corners[i], 0.05, camera_matrix, dist_coeffs, rvec, tvec);

        geometry_msgs::msg::Pose pose;
        pose.position.x = tvec[0];
        pose.position.y = tvec[1];
        pose.position.z = tvec[2];
        pose.orientation = rpyToQuaternion(rvec);

        detections.emplace_back(marker_ids[i], pose);
      }
    }

    // Publish results
    setOutput("detections", detections);
    RCLCPP_INFO(node_->get_logger(), "Detected %lu ArUco markers",
                detections.size());
    return BT::NodeStatus::SUCCESS;
  }

 private:
  geometry_msgs::msg::Quaternion rpyToQuaternion(const cv::Vec3d &rvec) {
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    geometry_msgs::msg::Quaternion q;

    if (trace > 0) {
      double s = 0.5 / std::sqrt(trace + 1.0);
      q.w = 0.25 / s;
      q.x = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
      q.y = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
      q.z = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
    } else {
      if (R.at<double>(0, 0) > R.at<double>(1, 1) &&
          R.at<double>(0, 0) > R.at<double>(2, 2)) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) -
                                   R.at<double>(1, 1) - R.at<double>(2, 2));
        q.w = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
        q.x = 0.25 * s;
        q.y = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        q.z = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
      } else if (R.at<double>(1, 1) > R.at<double>(2, 2)) {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) -
                                   R.at<double>(0, 0) - R.at<double>(2, 2));
        q.w = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
        q.x = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
        q.y = 0.25 * s;
        q.z = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
      } else {
        double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) -
                                   R.at<double>(0, 0) - R.at<double>(1, 1));
        q.w = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
        q.x = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
        q.y = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
        q.z = 0.25 * s;
      }
    }
    return q;
  }
};

#endif
