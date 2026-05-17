#ifndef BEHAVIOR_TREE__BT_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE__BT_SUBSCRIBER_NODE_HPP_

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
class BtSubscriberNode : public BT::ActionNodeBase {
 public:
  /**
   * @brief A nav2_behavior_tree::BtActionNode constructor
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  BtSubscriberNode(const std::string& xml_tag_name,
                   const std::string& topic_name,
                   const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf), topic_name_(topic_name) {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    topic_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "topic_timeout");
    getInput<std::chrono::milliseconds>("topic_timeout", topic_timeout_);

    std::string remapped_topic_name;
    if (getInput("topic_name", remapped_topic_name)) {
      topic_name_ = remapped_topic_name;
    }

    createSubscriber(topic_name_);

    // Give the derive class a chance to do any initialization
    RCLCPP_DEBUG(node_->get_logger(), "\"%s\" BtActionNode initialized",
                 xml_tag_name.c_str());
  }

  BtSubscriberNode() = delete;

  virtual ~BtSubscriberNode() {}

  /**
   * @brief Create instance of an subscriber
   * @param tipic_name Topic name to create subscriber for
   */
  void createSubscriber(const std::string& topic_name) {
    if (topic_name.empty()) {
      throw std::runtime_error("topic_name is empty");
    }

    rclcpp::SubscriptionOptions option;
    option.callback_group = callback_group_;

    // The callback will broadcast to all the instances of RosTopicSubNode<T>
    auto callback = [this](const std::shared_ptr<TopicT> msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_message_ = *msg;
      data_ready_ = true;
      time_topic_start_ = node_->now();
    };
    subscriber =
        node_->create_subscription<TopicT>(topic_name, 1, callback, option);
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::string>("topic_name", "Action name"),
        BT::InputPort<std::chrono::milliseconds>("topic_timeout")};
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
   * @brief Function to perform some user-defined operation on tick
   * Could do dynamic checks, such as getting updates to values on the
   * blackboard
   */
  /** Callback invoked in the tick. You must return either SUCCESS of FAILURE
   *
   * @param last_msg the latest message received, since the last tick.
   *                  Will be empty if no new message received.
   *
   * @return the new status of the Node, based on last_msg
   */
  virtual BT::NodeStatus on_tick() = 0;

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
    if (!subscriber) {
      throw BT::RuntimeError(
          "no subscriber was specified neither as default or "
          "in the ports");
    }

    // first step to be done only at the beginning of the Action
    if (status() == BT::NodeStatus::IDLE) {
      // setting the status to RUNNING to notify the BT Loggers (if any)
      setStatus(BT::NodeStatus::RUNNING);
      time_topic_start_ = node_->now();
    }
    callback_group_executor_.spin_some();
    auto elapsed = (node_->now() - time_topic_start_)
                       .to_chrono<std::chrono::milliseconds>();
    if (elapsed > topic_timeout_) {
      RCLCPP_WARN(node_->get_logger(), "Timed out while waiting for topic %s",
                  topic_name_.c_str());
      return BT::NodeStatus::FAILURE;
    }

    if (!data_ready_) {
      return BT::NodeStatus::RUNNING;
    }
    auto status = on_tick();
    data_ready_ = false;  // Reset flag for next tick

    return status;
  }

  std::optional<TopicT> getLastMessages() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (data_ready_) {
      data_ready_ = false;  // 
      return last_message_;
    }
    return std::nullopt;
  }

 protected:
  std::string topic_name_;
  using Subscriber = typename rclcpp::Subscription<TopicT>;
  std::shared_ptr<Subscriber> subscriber = nullptr;
  TopicT last_message_;
  bool data_ready_{false};
  std::mutex data_mutex_;

  // The node that will be used for any ROS operations
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::chrono::milliseconds topic_timeout_;
  rclcpp::Time time_topic_start_;
};

}  // namespace behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
