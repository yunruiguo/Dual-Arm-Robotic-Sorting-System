#ifndef ARM_CONTROL_ACTION_HPP
#define ARM_CONTROL_ACTION_HPP
#include "bt_action_node.hpp"
#include "grasp_msgs/action/arm_control.hpp"

using namespace behavior_tree;

class ArmControlNode : public BtActionNode<grasp_msgs::action::ArmControl> {
 public:
  ArmControlNode(const std::string &xml_tag_name,
                 const std::string &action_name,
                 const BT::NodeConfiguration &conf)
      : BtActionNode<grasp_msgs::action::ArmControl>(xml_tag_name, action_name,
                                                     conf) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
        BT::InputPort<std::vector<grasp_msgs::msg::ArmCommand>>(
            "arm_command", "Destination to Arm Command"),
        BT::OutputPort<grasp_msgs::msg::ArmCommand>("arm_position",
                                                    "Current Arm position"),
    });
  }
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override { getInput("arm_command", goal_.poses); }

  void on_wait_for_result(
      std::shared_ptr<const typename grasp_msgs::action::ArmControl::Feedback>
          feedback) override {
    if (feedback) {
      setOutput("arm_position", feedback->current_pose);
    }
  }

 private:
};

#endif
