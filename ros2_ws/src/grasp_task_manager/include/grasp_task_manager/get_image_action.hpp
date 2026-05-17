#ifndef GET_IMAGE__ACTION_HPP
#define GET_IMAGE__ACTION_HPP
#include <sensor_msgs/msg/image.hpp>

#include "bt_subscriber_node.hpp"

using namespace behavior_tree;

class GetImageNode : public BtSubscriberNode<sensor_msgs::msg::Image> {
 public:
  GetImageNode(const std::string &xml_tag_name, const std::string &topic_name,
               const BT::NodeConfiguration &conf)
      : BtSubscriberNode<sensor_msgs::msg::Image>(xml_tag_name, topic_name,
                                                  conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::OutputPort<sensor_msgs::msg::Image>("message_out",
                                                "Current Camera Image"),
    });
  }
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  BT::NodeStatus on_tick() override {
    auto msg = getLastMessages();
    if (msg.has_value()) {
      setOutput("message_out", msg.value());
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

 private:
};

#endif
