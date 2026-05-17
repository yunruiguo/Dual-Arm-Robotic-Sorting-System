#ifndef GET_IMAGE__ACTION_HPP
#define GET_IMAGE__ACTION_HPP
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "bt_synced_subscriber_node.hpp"

using namespace behavior_tree;

class GetSyncedImageAndPointCloud
    : public BtSyncedSubscriberNode<sensor_msgs::msg::Image,
                                    sensor_msgs::msg::PointCloud2,
                                    sensor_msgs::msg::CameraInfo> {
 public:
  GetSyncedImageAndPointCloud(const std::string &xml_tag_name,
                              const std::vector<std::string> &topic_names,
                              const BT::NodeConfiguration &conf)
      : BtSyncedSubscriberNode<sensor_msgs::msg::Image,
                               sensor_msgs::msg::PointCloud2,
                               sensor_msgs::msg::CameraInfo>(
            xml_tag_name, topic_names, conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts(
        {BT::OutputPort<sensor_msgs::msg::Image>("rgb_image",
                                                 "Current Camera Image"),
         BT::OutputPort<sensor_msgs::msg::PointCloud2>("point_cloud",
                                                       "Current PointCloud"),
         BT::OutputPort<sensor_msgs::msg::CameraInfo>("rgb_camera_info")});
  }
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus on_tick() override {
    auto msgs = getLastMessages();
    if (msgs.has_value()) {
      setOutput("rgb_image", std::get<0>(*msgs));
      setOutput("point_cloud", std::get<1>(*msgs));
      setOutput("rgb_camera_info", std::get<2>(*msgs));

      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

 private:
};

#endif
