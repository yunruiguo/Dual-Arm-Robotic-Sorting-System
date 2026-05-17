# Project Structure

This repository is organized by responsibility instead of by the original delivery bundle. Active source code, ROS 2 packages, experimental scripts, documentation, and archived legacy artifacts now live in separate top-level areas.

```text
.
├── README.md
├── src/vision/                         # Standalone ROS 2 Python perception nodes
├── ros2_ws/src/                        # Rebuildable ROS 2 C++ packages and messages
├── experiments/behavior_tree/          # Python behavior-tree and task-manager experiments
├── docs/
│   ├── deployment/                     # Edge-board, ROS 2, and MoveIt2 setup documents
│   ├── workflow/                       # Open-box workflow notes
│   └── architecture/                   # System architecture slides
└── archives/                           # Preserved legacy bundles for traceability
```

## Rename Map

| Legacy role | New location |
| --- | --- |
| ArUco perception script | `src/vision/aruco_pose_node.py` |
| Box keypoint and segmentation script | `src/vision/box_opening_keypoint_node.py` |
| Planar depth grasping script | `src/vision/planar_depth_grasp_node.py` |
| Legacy compressed workspace | `archives/legacy_robot_workspace.rar` |
| Edge-board installation guide | `docs/deployment/edge_board_system_install.pdf` |
| Ubuntu 22.04 ROS 2 deployment guide | `docs/deployment/edge_board_ubuntu22_ros2_humble_moveit2.docx` |
| Ubuntu 20.04 ROS 2 deployment guide | `docs/deployment/edge_board_ubuntu20_ros2_humble.docx` |
| Box-opening process document | `docs/workflow/open_box_process_v1.pdf` |
| System framework slides | `docs/architecture/system_framework_overview.pptx` |

## ROS 2 Workspace

The legacy archive contained a full ROS 2 workspace with generated `build/`, `install/`, and `log/` directories. The extracted source-only layout is now:

```text
ros2_ws/src/
├── grasp_common/grasp_util/
├── grasp_interfaces/grasp_msgs/
├── grasp_task_manager/
└── jaka_controller/
```

Build from `ros2_ws`:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
```

## Experiments

Behavior-tree prototypes and Python task-manager experiments are kept outside the build workspace:

```text
experiments/behavior_tree/
├── config/
├── launch/
├── test_behavior/
└── legacy/
```

`legacy/` contains duplicate or temporary scripts from the original bundle. They are preserved for comparison but should not be treated as the main entry points.

Experiment module names were normalized:

| Legacy role | New location |
| --- | --- |
| Main dual-arm controller experiment | `experiments/behavior_tree/test_behavior/dual_arm_controller.py` |
| Robotics-control dual-arm controller | `experiments/behavior_tree/test_behavior/robotics_control/dual_arm_controller.py` |
| Alternate robotics-control controller | `experiments/behavior_tree/test_behavior/robotics_control/dual_arm_controller_alt.py` |
| Box-opening controller variant | `experiments/behavior_tree/test_behavior/robotics_control/dual_arm_controller_open_box_hp.py` |
| Yunrui Guo controller variant | `experiments/behavior_tree/test_behavior/robotics_control/dual_arm_controller_yr.py` |
| Yunrui Guo controller copy | `experiments/behavior_tree/test_behavior/robotics_control/dual_arm_controller_yr_copy.py` |
