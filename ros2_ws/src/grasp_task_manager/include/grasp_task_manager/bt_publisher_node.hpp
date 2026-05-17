#ifndef BEHAVIOR_TREE__BT_PUBLISHER_NODE_HPP_
#define BEHAVIOR_TREE__BT_PUBLISHER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace behavior_tree {

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing an action based BT node
 * @tparam ActionT Type of action
 */
template <class TopicT>
class BtPublisherNode : public BT::ActionNodeBase {
 public:
  /**
   * @brief A nav2_behavior_tree::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BtPublisherNode(const std::string& xml_tag_name,
                  const std::string& topic_name,
                  const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf), topic_name_(topic_name) {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    std::string remapped_topic_name;
    if (getInput("topic_name", remapped_topic_name)) {
      topic_name_ = remapped_topic_name;
    }

    createPublisher(topic_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized",
                 xml_tag_name.c_str());
  }

  BtPublisherNode() = delete;

  virtual ~BtPublisherNode() {}

  /**
   * @brief Create instance of an subscriber
   * @param tipic_name Topic name to create subscriber for
   */
  void createPublisher(const std::string& topic_name) {
    if (topic_name.empty()) {
      throw std::runtime_error("topic_name is empty");
    }

    publisher = node_->create_publisher<TopicT>(topic_name, 1);
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("topic_name", "Action name")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  // Derived classes can override any of the following methods to hook into the
  // processing for the action: on_tick, on_wait_for_result, and on_success

  /**
   * @brief setMessage is a callback invoked in tick to allow the user to pass
   * the message to be published.
   *
   * @param msg the message.
   * @return  return false if anything is wrong and we must not send the
   * message. the Condition will return FAILURE.
   */
  virtual bool setMessage(TopicT& msg) = 0;

  /** latch the message that has been processed. If returns false and no new
   * message is received, before next call there will be no message to process.
   * If returns true, the next call will process the same message again, if no
   * new message received.
   *
   * This can be equated with latched vs non-latched topics in ros 1.
   *
   * @return false will clear the message after ticking/processing.
   */
  virtual bool latchLastMessage() const { return false; }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override {
    if (!publisher) {
      throw BT::RuntimeError(
          "no subscriber was specified neither as default or "
          "in the ports");
    }

    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);
    }
    TopicT msg;
    if (!setMessage(msg)) {
      return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
  }

 protected:
  std::string topic_name_;
  using Publisher = typename rclcpp::Publisher<TopicT>;
  std::shared_ptr<Publisher> publisher = nullptr;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
};

}  // namespace behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
