#ifndef BEHAVIOR_TREE__BT_SYNCED_SUBSCRIBER_NODE_HPP_
#define BEHAVIOR_TREE__BT_SYNCED_SUBSCRIBER_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace behavior_tree {

using namespace std::chrono_literals;  // NOLINT

using namespace std::chrono_literals;  // NOLINT

/**
 * @brief Abstract class representing a multi-topic subscriber BT node
 * @tparam MessageTypes Variadic template of message types
 */
template <typename... MessageTypes>
class BtSyncedSubscriberNode : public BT::ActionNodeBase {
 public:
  using SyncPolicy =
      message_filters::sync_policies::ApproximateTime<MessageTypes...>;
  using CallbackFunction = std::function<void(const MessageTypes&...)>;

  BtSyncedSubscriberNode(const std::string& xml_tag_name,
                         const std::vector<std::string>& topic_names,
                         const BT::NodeConfiguration& conf)
      : BT::ActionNodeBase(xml_tag_name, conf), topic_names_(topic_names) {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");

    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    topic_timeout_ =
        config().blackboard->template get<std::chrono::milliseconds>(
            "topic_timeout");
    getInput<std::chrono::milliseconds>("topic_timeout", topic_timeout_);

    std::vector<std::string> remapped_topic_names;
    if (getInput("topic_names", remapped_topic_names)) {
      topic_names_ = remapped_topic_names;
    }
    if (topic_names_.empty() ||
        topic_names_.size() != sizeof...(MessageTypes)) {
      throw std::runtime_error(
          "The number of topic names must match the number of message types");
    }

    createSubscribers(topic_names_);

    // Log initialization
    RCLCPP_DEBUG(node_->get_logger(),
                 "\"%s\" BtMultiSubscriberNode initialized",
                 xml_tag_name.c_str());
  }

  BtSyncedSubscriberNode() = delete;
  virtual ~BtSyncedSubscriberNode() {}

  /**
   * @brief Create subscribers for multiple topics
   * @param topic_names List of topic names
   */
  void createSubscribers(const std::vector<std::string>& topic_names) {
    if (topic_names.empty()) {
      throw std::runtime_error("No topic names provided");
    }
    if (topic_names.size() != sizeof...(MessageTypes)) {
      throw std::runtime_error(
          "Mismatch between topic names and message types");
    }

    size_t index = 0;

    // 
    (createSubscriber<MessageTypes>(topic_names[index++]), ...);

    //  subscribers_ 
    if (subscribers_.size() != sizeof...(MessageTypes)) {
      throw std::runtime_error("Subscribers were not correctly created");
    }

    // 
    initializeSynchronizer(std::make_index_sequence<sizeof...(MessageTypes)>{});
  }

  std::optional<std::tuple<typename MessageTypes::ConstSharedPtr...>>
  getLastMessages() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (data_ready_) {
      data_ready_ = false;  // 
      return last_messages_;
    }
    return std::nullopt;
  }

  /**
   * @brief Any subclass of BtActionNode that accepts parameters must provide a
   * providedPorts method and call providedBasicPorts in it.
   * @param addition Additional ports to add to BT port list
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedBasicPorts(BT::PortsList addition) {
    BT::PortsList basic = {
        BT::InputPort<std::vector<std::string>>("topic_names", "topics name"),
        BT::InputPort<std::chrono::milliseconds>("topic_timeout")};
    basic.insert(addition.begin(), addition.end());

    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() { return providedBasicPorts({}); }

  /**
   * @brief Main tick function of the node
   * @return Node status: SUCCESS, FAILURE, or RUNNING
   */
  BT::NodeStatus tick() override {
    if (status() == BT::NodeStatus::IDLE) {
      setStatus(BT::NodeStatus::RUNNING);
      time_topic_start_ = node_->now();
    }

    callback_group_executor_.spin_some();
    auto elapsed = (node_->now() - time_topic_start_)
                       .to_chrono<std::chrono::milliseconds>();

    if (elapsed > topic_timeout_) {
      RCLCPP_WARN(node_->get_logger(), "Timed out while waiting for topics");
      return BT::NodeStatus::FAILURE;
    }

    if (!data_ready_) {
      return BT::NodeStatus::RUNNING;
    }

    auto status = on_tick();
    data_ready_ = false;  // Reset flag for next tick
    return status;
  }

 private:
  template <typename MessageType>
  void createSubscriber(const std::string& topic_name) {
    if (topic_name.empty()) {
      throw std::runtime_error("Topic name is empty");
    }
    auto subscriber =
        std::make_shared<message_filters::Subscriber<MessageType>>(node_,
                                                                   topic_name);
    // 
    if (!subscriber) {
      throw std::runtime_error("Failed to create subscriber for topic: " +
                               topic_name);
    }
    subscribers_.emplace_back(subscriber);
  }

  template <size_t... Indices>
  void initializeSynchronizer(std::index_sequence<Indices...>) {
    synchronizer_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10),
        *std::static_pointer_cast<message_filters::Subscriber<
            std::tuple_element_t<Indices, std::tuple<MessageTypes...>>>>(
            subscribers_[Indices])...);

    //  Lambda 
    synchronizer_->registerCallback(
        [this](const typename MessageTypes::ConstSharedPtr&... msgs) {
          this->callback(msgs...);
        });
  }

  void callback(const typename MessageTypes::ConstSharedPtr&... msgs) {
    RCLCPP_INFO(node_->get_logger(), "Synchronized messages received");
    std::lock_guard<std::mutex> lock(data_mutex_);
    last_messages_ = std::make_tuple(msgs...);
    data_ready_ = true;
  }

 protected:
  /**
   * @brief User-defined processing for received synchronized messages
   * @return Node status: SUCCESS or FAILURE
   */
  virtual BT::NodeStatus on_tick() = 0;

  std::vector<std::string> topic_names_;
  std::tuple<std::shared_ptr<MessageTypes>...> last_messages_;
  std::tuple<std::shared_ptr<message_filters::Subscriber<MessageTypes>>...>
      subscribers_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> synchronizer_;
  bool data_ready_{false};
  std::mutex data_mutex_;

  // ROS 2 specific members
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  std::chrono::milliseconds topic_timeout_;
  rclcpp::Time time_topic_start_;
};

}  // namespace behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__BT_ACTION_NODE_HPP_
