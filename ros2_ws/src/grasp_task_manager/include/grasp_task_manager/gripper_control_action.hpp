#ifndef GRIPPER_CONTROL_ACTION_HPP
#define ARM_CONTROL_ACTION_HPP
#include "bt_action_node.hpp"
#include "grasp_msgs/action/gripper_control.hpp"

using namespace behavior_tree;

class GripperControlNode
    : public BtActionNode<grasp_msgs::action::GripperControl> {
 public:
  GripperControlNode(const std::string &xml_tag_name,
                     const std::string &action_name,
                     const BT::NodeConfiguration &conf)
      : BtActionNode<grasp_msgs::action::GripperControl>(xml_tag_name,
                                                         action_name, conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<grasp_msgs::msg::GripperCommand>(
            "gripper_command", "Destination to gripper Command"),
        BT::OutputPort<grasp_msgs::msg::GripperCommand>(
            "gripper_position", "Current Gripper position"),
    });
  }
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override { getInput("gripper_command", goal_.pose); }

  void on_wait_for_result(
      std::shared_ptr<
          const typename grasp_msgs::action::GripperControl::Feedback>
          feedback) override {
    if (feedback) {
      setOutput("gripper_position", feedback->current_pose);
    }
  }

 private:
};

#endif
