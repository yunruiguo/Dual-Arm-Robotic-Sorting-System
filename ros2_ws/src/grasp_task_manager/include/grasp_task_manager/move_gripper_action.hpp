#ifndef MOVE_GRIPPER_ACTION_HPP
#define MOVE_GRIPPER_ACTION_HPP
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
        BT::InputPort<double>("position", "Current Gripper position"),
        BT::InputPort<double>("force", "Current Gripper force"),
    });
  }
  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override {
    double position;
    double force;
    getInput("position", position);
    getInput("force", force);

    goal_.pose.pos = position;
    goal_.pose.force = force;
  }

  void on_wait_for_result(
      std::shared_ptr<
          const typename grasp_msgs::action::GripperControl::Feedback>
          feedback) override {
    if (feedback) {
    }
  }

 private:
};

#endif
