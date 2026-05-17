import rclpy
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.exceptions import ROSInterruptException
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from grasp_msgs.msg import ArmCommand, GripperCommand
from grasp_msgs.action import ArmControl, GripperControl
from action_msgs.msg import GoalStatus
from rclpy.subscription import Subscription
from grasp_msgs.msg import FeedBackMsg

class ArmFeedbackMsg:
    def __init__(self):
        self.pose = ArmCommand()
        self.distance_remaining = 0.0
        self.is_running = False
        self.success = False

    def reset(self):
        """Reset the feedback message fields to their default values."""
        self.pose = ArmCommand()
        self.distance_remaining = 0.0
        self.is_running = False
        self.success = False


class GripperFeedbackMsg:
    def __init__(self):
        self.pose = GripperCommand()
        self.success = False

    def reset(self):
        """Reset the feedback message fields to their default values."""
        self.pose = GripperCommand()
        self.distance_remaining = 0.0
        self.is_running = False
        self.success = False


class ArmController:
    def __init__(self, node, name_prefix):
        self.node = node
        self.name_prefix = name_prefix
        self.arm_feedback = ArmFeedbackMsg()
        self.gripper_feedback = GripperFeedbackMsg()
        self.arm_cb_group = MutuallyExclusiveCallbackGroup()
        self.gripper_cb_group = MutuallyExclusiveCallbackGroup()
        self.feedback_cb_group = MutuallyExclusiveCallbackGroup()
        self.node.get_logger().info(f"Created arm action for {name_prefix}")
        # Create the subscription with a custom callback group
        self.arm_feedback_sub = self.node.create_subscription(
            FeedBackMsg,
            f"{name_prefix}/feedback_states",
            self.arm_feedback_callback,
            1,
            callback_group=self.feedback_cb_group
        )
        
        try:
            self.arm_action_client = ActionClient(node, ArmControl,
             f"{name_prefix}/arm_control",
             callback_group=self.arm_cb_group)
            self.node.get_logger().info(f"Created arm action client for {name_prefix}")
        except ROSInterruptException as e:
            self.node.get_logger().error(f"Error creating arm action client: {e}")
            raise

        self.node.get_logger().info(f"Created gripper action for {name_prefix}")
        
        try:
            self.gripper_action_client = ActionClient(node, GripperControl, 
            f"{name_prefix}/gripper_control",
            callback_group=self.gripper_cb_group)
            self.node.get_logger().info(f"Created gripper action client for {name_prefix}")
        except ROSInterruptException as e:
            self.node.get_logger().error(f"Error creating gripper action client: {e}")
            raise
        
        self.node.get_logger().info(f"Created controller for {name_prefix}")


    def wait_for_action_servers(self):
        """ Action ."""
        self.node.get_logger().info(f"{self.name_prefix}")
        while not self.arm_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info(f"{self.name_prefix}...")

        while not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().info(f"{self.name_prefix}...")

    def arm_feedback_callback(self, msg):
        """Callback for handling arm feedback."""
        #self.node.get_logger().info(f"Received feedback: {msg}")
        # If arm_feedback is not running, update its pose
        if not self.arm_feedback.is_running:
            # Update TCP position
            self.arm_feedback.pose.tcp.pos.x = msg.tcp_pose[0]
            self.arm_feedback.pose.tcp.pos.y = msg.tcp_pose[1]
            self.arm_feedback.pose.tcp.pos.z = msg.tcp_pose[2]

            # Update TCP Euler angles
            self.arm_feedback.pose.tcp.euler.x = msg.tcp_pose[3]
            self.arm_feedback.pose.tcp.euler.y = msg.tcp_pose[4]
            self.arm_feedback.pose.tcp.euler.z = msg.tcp_pose[5]

            # Update joint positions
            self.arm_feedback.pose.joint.joint = list(msg.joint_pose)

    def execute_arm(self, poses):
        """Send an action goal to move the arm."""
        self.arm_feedback.reset()
        goal_msg = ArmControl.Goal()
        goal_msg.poses = poses

        if not self.arm_action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(f"Action server for {self.name_prefix} arm is unavailable")
            return False
        
        self.node.get_logger().info(f"Sending {self.name_prefix} arm goal")
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.arm_goal_callback)
        return True

    def execute_gripper(self, command):
        """Send an action goal to control the gripper."""
        self.gripper_feedback.reset()
        goal_msg = GripperControl.Goal()
        goal_msg.pose = command

        if not self.gripper_action_client.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().error(f"Action server for {self.name_prefix} gripper is unavailable")
            return False
        
        self.node.get_logger().info(f"Sending {self.name_prefix} gripper goal")
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.gripper_goal_callback)
        return True

    def arm_goal_callback(self, future):
        """Handle the response from the arm action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name_prefix} arm goal was rejected")
            self.arm_feedback.success = False
            return
        
        self.node.get_logger().info(f"{self.name_prefix} arm goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def gripper_goal_callback(self, future):
        """Handle the response from the gripper action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name_prefix} gripper goal was rejected")
            self.gripper_feedback.success = False
            return
        
        self.node.get_logger().info(f"{self.name_prefix} gripper goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_callback)

    def arm_result_callback(self, future):
        """Handle the result from the arm action server."""
        result = future.result().result
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name_prefix} arm goal succeeded")
            self.arm_feedback.success = True
        else:
            self.node.get_logger().warn(f"{self.name_prefix} arm goal failed or aborted")
            self.arm_feedback.success = False

    def gripper_result_callback(self, future):
        """Handle the result from the gripper action server."""
        result = future.result().result
        if future.result().status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info(f"{self.name_prefix} gripper goal succeeded")
            self.gripper_feedback.success = True
        else:
            self.node.get_logger().warn(f"{self.name_prefix} gripper goal failed or aborted")
            self.gripper_feedback.success = False


class GraspTaskManager(Node):
    def __init__(self):
        super().__init__('grasp_task_manager_node')
        try:
            self.cam_pose = np.genfromtxt('real/right_camera_pose.txt', delimiter=' ')
            self.top_rightcamera_pose = np.genfromtxt('real/top_rightcamera_pose.txt', delimiter=' ')
            self.top_leftcamera_pose = np.genfromtxt('real/top_leftcamera_pose.txt', delimiter=' ')
        except Exception as e:
            print("Error reading file:", e)
        self.center_point=[0.0,0.0,0.0]
        self.middle_left =[0.0,0.0,0.0]
        self.middle_right=[0.0,0.0,0.0] 
        self.arm_controllers = [
            ArmController(self, "left"),
            ArmController(self, "right")
        ]
        for controller in self.arm_controllers:
            controller.wait_for_action_servers()

        self.jointTo(1, [230.416*np.pi/180,105.559*np.pi/180,38.741*np.pi/180,93.193*np.pi/180,-123.021*np.pi/180,-229.462*np.pi/180])
        self.moveTo(1, [517.683,-213.007,60.0,3.1416,0.0,1.5808])
        # self.moveTo(0, [-479.762,-85.60666223,18.0,3.1416,0.0,0.0])
        
        subscription = self.create_subscription(Float64MultiArray,'right/grasppose', self.subscription_callback,10)
        printed_once = True
        while printed_once and rclpy.ok():
            rclpy.spin_once(self) 
            if self.center_point[0]!=0.0:
                pose3=self.get_pose(self.middle_right)
                pose2=self.get_pose(self.center_point)
                pose1=self.get_pose(self.middle_left)
                poseleft=self.get_left_pinpose(self.center_point)
                
                pose4=self.get_pose(self.left_point)
                pose5=self.get_pose(self.right_point)
                pose2[2]=(pose4[2]+pose5[2])/2
                # result = self.robot.linear_move(pose5,0,True,20)
                # 
                pose1[2] = pose2[2]+2.5
                pose3[2] = pose2[2]-1.5
                # pose5=self.transform_with_combined(pose5,np.radians(0),np.radians(0),np.radians(137),8)
                # result = self.robot.linear_move(pose5,0,True,20)
                #  BA  BC
                vec_BA = pose1 - pose2
                vec_BC = pose3 - pose2
                print("vec_BC:",vec_BC)

                #  k
                k = 1.1  # modified

                #  d  e 
                d = pose1 + k * vec_BA
                e = pose3 + k * vec_BC
                vec_top_left = pose5 - pose3
                vec_top_right = pose4 - pose3
                vec_middle_right = pose5 - pose2
                vec_middle_left = pose4 - pose2
                left_circl = pose4+  vec_middle_left
                right_circl = pose5 + vec_middle_right
                top_left = pose3 - 0.95* vec_top_left
                top_right = pose3 - 0.95* vec_top_right
                vec_down_left = pose5 - pose1
                vec_down_right = pose4 - pose1
                down_left = pose1 - 0.87* vec_down_left
                down_right = pose1 - 0.87* vec_down_right
                                # 
                position1 = np.array(pose2[:2])  # [x1, y1, z1]
                position2 = np.array(right_circl[:2])  # [x2, y2, z2]

                # 
                distance = np.linalg.norm(position1 - position2)
                print("distance",distance)
                top_left[5]=0
                top_right[5]=0
                down_left[5]=0
                down_right[5]=0
                # result = self.robot.linear_move(pose4,0,True,20)
                # 
                print(" e :", e)
                print(" pose3 :", pose3)
                print(" pose2 :", pose2)
                print(" pose1 :", pose1)
                print(" poseleft :", poseleft)
                print(" pose4 :", pose4)
                print(" pright :", pose5)
                print(" d :", d)
                print(" top_left :", top_left)
                print(" top_right :", top_right)
                print("left_circle:", left_circl)
                print("right_circle:", right_circl)

    def calculate_rotated_point_and_orientation_euler(self,start_pose, center, rotation_axis, theta):
        """
         (x, y, z, r_x, r_y, r_z) Parameters,.

        :param start_pose:  (x, y, z, r_x, r_y, r_z) ( + ,xyz)
        :param center:  (x, y, z)
        :param rotation_axis:  (x, y, z)
        :param theta:  ()
        :return:  (x', y', z', r_x', r_y', r_z')
        """
        # 
        p_start = np.array(start_pose[:3])  #  (x, y, z)
        euler_start = start_pose[3:]  #  (r_x, r_y, r_z)
        p_center = np.array(center)  # 
        axis = np.array(rotation_axis)  # 

        # 
        u = axis / np.linalg.norm(axis)

        #  ( theta)
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        u_x, u_y, u_z = u
        R_axis = np.array([
            [cos_theta + u_x**2 * (1 - cos_theta), u_x * u_y * (1 - cos_theta) - u_z * sin_theta, u_x * u_z * (1 - cos_theta) + u_y * sin_theta],
            [u_y * u_x * (1 - cos_theta) + u_z * sin_theta, cos_theta + u_y**2 * (1 - cos_theta), u_y * u_z * (1 - cos_theta) - u_x * sin_theta],
            [u_z * u_x * (1 - cos_theta) - u_y * sin_theta, u_z * u_y * (1 - cos_theta) + u_x * sin_theta, cos_theta + u_z**2 * (1 - cos_theta)]
        ])

        # 
        p_local = p_start - p_center  # 
        p_rot = R_axis @ p_local  # 
        p_end = p_rot + p_center  # 

        # 
        R_start = R.from_euler('xyz', euler_start).as_matrix()

        # 
        R_end = R_axis @ R_start  # 
        euler_end = R.from_matrix(R_end).as_euler('xyz')  # 
        combined = np.concatenate((p_end, euler_end))
        return combined

    def calculate_rotation_to_target_z_axis(self,start_pose, center, rotation_axis, tolerance=1e-1):
        """
         Z .
        
        :param start_pose:  (x, y, z, r_x, r_y, r_z), (XYZ )
        :param center:  (cx, cy, cz)
        :param rotation_axis:  (ux, uy, uz)
        :param tolerance: 
        :return:  θ ()
        """
        # 
        p_start = np.array(start_pose[:3])  # 
        euler_start = start_pose[3:]  # 
        p_center = np.array(center)  # 
        axis = np.array(rotation_axis)  # 

        # 
        u = axis / np.linalg.norm(axis)

        # 
        target_vector = p_center - p_start  # 
        target_vector = target_vector / np.linalg.norm(target_vector)  # 

        #  Z 
        R_start = R.from_euler('xyz', euler_start).as_matrix()  # 
        z_start = R_start[:, 2]  #  Z 

        # 
        def find_theta(z_start, target_vector, u):
            for theta in np.linspace(0, 2 * np.pi, 1000):  #  [0, 2π] 
                # 
                R_axis = R.from_rotvec(theta * u).as_matrix()
                z_end = R_axis @ z_start  #  Z 

                #  z_end  target_vector 
                if np.abs(np.dot(z_end, target_vector) - 1) < tolerance:
                    return theta
            return None

        # Find tcp_pose = [tcp_position.x, tcp_position.y, tcp_position.z, tcp_euler.x, tcp_euler.y, tcp_euler.z]
        theta = find_theta(z_start, target_vector, u)

        if theta is None:
            raise ValueError(",")

        return theta
    def subscription_callback(self,grasppose):
        self.center_point = list(grasppose.data[0:3])
        self.middle_left = list(grasppose.data[3:6])
        self.middle_right = list(grasppose.data[6:9])
        self.left_point = list(grasppose.data[9:12])
        self.right_point = list(grasppose.data[12:15])
    def get_pose(self,xyz):
        rpy_xyz=(180,0,90)
        transformation_matrix = np.eye(4)
        transformation_matrix = self.pose_and_quaternion_to_matrix(xyz,rpy_xyz)
        # (pose2[2] = (pose1[2] + pose2[2]+pose3[2])/3,x,y,zx=0,y=0,ｚ0.2)
                #  TCP 
        tcp_position = self.arm_controllers[1].arm_feedback.pose.tcp.pos
        tcp_euler = self.arm_controllers[1].arm_feedback.pose.tcp.euler
        tool_pose = [tcp_position.x, tcp_position.y, tcp_position.z, tcp_euler.x, tcp_euler.y, tcp_euler.z]
        print(tool_pose)
        if tool_pose is None:
            self.get_logger().error("TCP pose is not available yet.")
            return None
        tool_to_base_translation=self.pose__matrix(tool_pose)
        camera_to_base_translation = np.dot(tool_to_base_translation,self.cam_pose)
        object_to_base_translation = np.dot(camera_to_base_translation,transformation_matrix)
        #  (x, y, z)
        current_translation = np.array(object_to_base_translation[:3, 3])
        # 
        rotation_matrix = object_to_base_translation[:3, :3]
        end_effector_direction= rotation_matrix[:,2]
        # current_translation = current_translation - 205 * end_effector_direction
        rpy_xyz = np.array((170*np.pi/180,10*np.pi/180,1.5708))
       #   x+5 y+7 z+2
       #  x+4.5 y+5.5 z+87
       # 85 84.3 20.3,19.5
        current_translation [0]+=5
        current_translation [1]+=7
        current_translation [2]+=86.5
        current_translation [2]+=2
        knif_to_base_translation = self.pose_and_quaternion_to_matrix(current_translation ,rpy_xyz)
        R_tool = knif_to_base_translation[:3, :3]
        theta_x = np.radians(0)  #  x  (: )
        theta_y = np.radians(0)  #  y  (: )
        theta_z = np.radians(-43)  #  z  (: )
        L = 8  # 

        # 
        cos_x, sin_x = np.cos(theta_x), np.sin(theta_x)
        cos_y, sin_y = np.cos(theta_y), np.sin(theta_y)
        cos_z, sin_z = np.cos(theta_z), np.sin(theta_z)

        # 
        d_local = np.array([
            cos_y * cos_z,
            sin_x * sin_y * cos_z + cos_x * sin_z,
            cos_x * sin_y * cos_z - sin_x * sin_z
        ])

        # 
        d_world = R_tool @ d_local  # 
        d_world /= np.linalg.norm(d_world)  # 
 
        # 
        P_end = current_translation + L * d_world
        # P_end[2]=P_end[2]-16
        print(":",P_end)

        combined = np.concatenate((current_translation, rpy_xyz))
    
        return combined
    def get_left_pinpose(self,xyz):
#         depth_to_color = np.array([
#     [0.99992,-0.0116565,-0.00493683,0.0148483],
#     [0.0116607,0.999932,0.000819961,-0.000120824],
#     [0.00492694,-0.000877462,0.999987,7.04899e-05],
#     [0., 0., 0., 1.]
# ])
    
        rpy_xyz=(180,0,90)
        transformation_matrix = np.eye(4)
        transformation_matrix = self.pose_and_quaternion_to_matrix(xyz,rpy_xyz)
        # (pose2[2] = (pose1[2] + pose2[2]+pose3[2])/3,x,y,zx=0,y=0,ｚ0.2)
        #  TCP 
        tcp_position = self.arm_controllers[1].arm_feedback.pose.tcp.pos
        tcp_euler = self.arm_controllers[1].arm_feedback.pose.tcp.euler
        tool_pose = [tcp_position.x, tcp_position.y, tcp_position.z, tcp_euler.x, tcp_euler.y, tcp_euler.z]
        print(tool_pose)
        if tool_pose is None:
            self.get_logger().error("TCP pose is not available yet.")
            return None
        tool_to_base_translation=self.pose__matrix(tool_pose)
        camera_to_base_translation = np.dot(tool_to_base_translation,self.cam_pose)
        right_camera_to_top_translation = np.dot(self.inverse_transformation_matrix(self.top_rightcamera_pose) ,camera_to_base_translation)
        right_camera_to_leftbase_translation = np.dot(self.top_leftcamera_pose ,right_camera_to_top_translation)
        object_to_base_translation = np.dot(right_camera_to_leftbase_translation,transformation_matrix)
        #print("object_to_base",object_to_base_translation)
        #  (x, y, z)
        current_translation = np.array(object_to_base_translation[:3, 3])
        # 
        rotation_matrix = object_to_base_translation[:3, :3]
        end_effector_direction= rotation_matrix[:,2]
        # rpy_xyz = np.array((1.5515958566998687, 0.8202967590417055, -2.3736464631660774))  
        rpy_xyz = np.array((2.77568823928427, -0.3201011841616206, 0.03134225891614126))  
        current_translation [0]+=11.4
        current_translation [1]+=1.04
        current_translation [2]+=-2.45
        knif_to_base_translation = self.pose_and_quaternion_to_matrix(current_translation ,rpy_xyz)
        R_tool = knif_to_base_translation[:3, :3]
        theta_x = np.radians(0)  #  x  (: )
        theta_y = np.radians(0)  #  y  (: )
        theta_z = np.radians(45)  #  z  (: )
        L = 42  # 

        # 
        cos_x, sin_x = np.cos(theta_x), np.sin(theta_x)
        cos_y, sin_y = np.cos(theta_y), np.sin(theta_y)
        cos_z, sin_z = np.cos(theta_z), np.sin(theta_z)

        # 
        d_local = np.array([
            cos_y * cos_z,
            sin_x * sin_y * cos_z + cos_x * sin_z,
            cos_x * sin_y * cos_z - sin_x * sin_z
        ])

        # 
        d_world = R_tool @ d_local  # 
        d_world /= np.linalg.norm(d_world)  # 
 
        # 
        P_end = current_translation + L * d_world
        # P_end[2]=P_end[2]-16
        print(P_end)
        combined = np.concatenate((P_end, rpy_xyz))
        
        return combined
    def get_targetpose(self, transformation_matrix,
                       rpy_xyz=(180, 0, 90),
                       is_radians=False, 
                       use_calibration=False, 
                       mode="target",
                       arm_index=0):
        # 
        depth_to_color = np.array([
            [0.99992, -0.0116565, -0.00493683, 0.0148483],
            [0.0116607, 0.999932, 0.000819961, -0.000120824],
            [0.00492694, -0.000877462, 0.999987, 7.04899e-05],
            [0., 0., 0., 1.]
        ])

        # 
        calibration = np.array([
            [0.9998476951563913, 0, -0.01745240643728351, 0],
            [0, 1, 0, 0],
            [0.01745240643728351, 0, 0.9998476951563913, 0],
            [0., 0., 0., 1.]
        ])

       #  TCP 
        tcp_position = self.arm_controllers[1].arm_feedback.pose.tcp.pos
        tcp_euler = self.arm_controllers[1].arm_feedback.pose.tcp.euler
        tool_pose = [tcp_position.x, tcp_position.y, tcp_position.z, tcp_euler.x, tcp_euler.y, tcp_euler.z]
        print(tool_pose)
        if tool_pose is None:
            self.get_logger().error("TCP pose is not available yet.")
            return None
        tool_to_base_translation=self.pose__matrix(tool_pose)
        # 
        if len(transformation_matrix) == 6:
            translation = transformation_matrix[:3]
            rotation_angles = transformation_matrix[3:]
            # 
            if is_radians:
                r = R.from_euler('xyz', rotation_angles, degrees=False)
            else:
                rotation_angles_in_radians = np.radians(rotation_angles)
                r = R.from_euler('xyz', rotation_angles_in_radians, degrees=False)
            rotation_matrix = r.as_matrix()
            deep_cam2tool = np.dot(self.cam_pose, depth_to_color)
            rotation_matrix_1 = deep_cam2tool[:3, :3]
            rot_z_matrix = np.array([[0.70711, -0.70711, 0], [0.70711, 0.70711, 0], [0, 0, 1]])  # 45
            point_to_tool = np.dot(rotation_matrix_1, rot_z_matrix)
            deep_cam2tool[:3, :3] = point_to_tool
            depth_to_base = np.dot(tool_to_base_translation, deep_cam2tool)
        else:
            translation = transformation_matrix
            rotation_angles = rpy_xyz
            # , camera_to_base_translation
            depth_to_base = np.dot(tool_to_base_translation, self.cam_pose)
        # 
        if mode == "target":
            rotation_translation_4x4 = self.pose_and_quaternion_to_matrix(translation,rotation_angles)
            # 
            object_to_base_translation = np.dot(depth_to_base, rotation_translation_4x4)

            #  (x, y, z)
            current_translation = object_to_base_translation[:3, 3]

            # 
            rotation_matrix = object_to_base_translation[:3, :3]

            # 
            r = R.from_matrix(rotation_matrix)
            euler_angles = r.as_euler('xyz', degrees=True)

            # 
            if use_calibration:
                object_to_base_translation = np.dot(object_to_base_translation, calibration)
                current_translation = object_to_base_translation[:3, 3]
                rotation_matrix = object_to_base_translation[:3, :3]
                r = R.from_matrix(rotation_matrix)
                euler_angles = r.as_euler('xyz', degrees=True)

            # 
            combined = np.concatenate((current_translation, euler_angles), axis=0)
            return combined
        elif mode == "left_pin":
            right_camera_to_top_translation = np.dot(self.inverse_transformation_matrix(self.top_rightcamera_pose), depth_to_base)
            right_camera_to_leftbase_translation = np.dot(self.top_leftcamera_pose, right_camera_to_top_translation)
            object_to_base_translation = np.dot(right_camera_to_leftbase_translation, rotation_translation_4x4)

            # 
            current_translation = object_to_base_translation[:3, 3]
            rpy_xyz = np.array((2.77568823928427, -0.3201011841616206, 0.03134225891614126))
            current_translation[0] += 11.4
            current_translation[1] += 1.04
            current_translation[2] += -2.45

            combined = np.concatenate((current_translation, rpy_xyz))
            return combined
    
    def pose_and_quaternion_to_matrix(self,position, euler):
        #  X 
        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(euler[0]), -math.sin(euler[0])],
            [0, math.sin(euler[0]), math.cos(euler[0])]
        ])

        #  Y 
        Ry = np.array([
            [math.cos(euler[1]), 0, math.sin(euler[1])],
            [0, 1, 0],
            [-math.sin(euler[1]), 0, math.cos(euler[1])]
        ])

        #  Z 
        Rz = np.array([
            [math.cos(euler[2]), -math.sin(euler[2]), 0],
            [math.sin(euler[2]), math.cos(euler[2]), 0],
            [0, 0, 1]
        ])

        #  R = Rz * Ry * Rx
        rotation_matrix = np.dot(np.dot(Rz, Ry), Rx)

        #  4x4 
        transformation_matrix = np.eye(4)
        
        # Set
        transformation_matrix[:3, :3] = rotation_matrix
        
        # Set
        transformation_matrix[:3, 3] = position
        return transformation_matrix
    def pose__matrix(self,cartesian_pose):
        x, y, z, rx, ry, rz = cartesian_pose

        # 
        rotation = R.from_euler('xyz', [rx, ry, rz])
        rotation_matrix = rotation.as_matrix()

        # 
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = [x, y, z]

        return transformation_matrix
    
    def transform_with_combined(self, combined, theta_x, theta_y, theta_z, L):
        """
        Parameters(combined),,.
        ,.
        
        Parameters:
        - combined: np.ndarray, , [x, y, z, roll, pitch, yaw].
        - theta_x: float,  x ().
        - theta_y: float,  y ().
        - theta_z: float,  z ().
        - L: float, .
         -2.706794133013487, 0.36197585591144077, 1.6778486254083171
          2.77568823928427, -0.3201011841616206, 0.03134225891614126

        
        Returns:
        - new_combined: np.ndarray, , [x, y, z, roll, pitch, yaw].
        """
        # 
        current_translation = combined[:3]
        print("current_translation",current_translation)
        current_rpy = combined[3:]
        print("current_rpy",current_rpy)

        #  pose_and_quaternion_to_matrix 
        T_tool_to_world = self.pose_and_quaternion_to_matrix(current_translation, current_rpy)
        R_tool = T_tool_to_world [:3, :3]
        
        # ()
        cos_x, sin_x = np.cos(theta_x), np.sin(theta_x)
        cos_y, sin_y = np.cos(theta_y), np.sin(theta_y)
        cos_z, sin_z = np.cos(theta_z), np.sin(theta_z)
        direction_local = np.array([
            cos_y * cos_z,
            sin_x * sin_y * cos_z + cos_x * sin_z,
            cos_x * sin_y * cos_z - sin_x * sin_z
        ])

        # 
        d_world = R_tool @ direction_local  # 
        d_world /= np.linalg.norm(d_world)  # 
        rpy_xyz=(-2.706794133013487, 0.36197585591144077, 1.6778486254083171)
        # 
        P_end = current_translation + L * d_world
        # P_end[2]=P_end[2]-16
        print(P_end)
        combined = np.concatenate((P_end, rpy_xyz))
        return combined

    def rotation_matrix_to_euler(self, R):
        """
         (roll, pitch, yaw).
        """
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

        return np.array([roll, pitch, yaw])
    def inverse_transformation_matrix(self,T):
        R = T[:3, :3]
        t = T[:3, 3]

        # 
        R_inv = R.T

        # 
        t_inv = -np.dot(R_inv, t)


        # 
        T_inv = np.identity(4)
        T_inv[:3, :3] = R_inv
        T_inv[:3, 3] = t_inv

        return T_inv


    def jointTo(self,index, joint):
         # Example commands
        arm_command = ArmCommand()
        #
        arm_command.type = arm_command.JOINT_TYPE
        arm_command.joint.joint = joint
        self.execute_arm_control(index, arm_command)

    def moveTo(self,index, tcp):
         # Example commands
        arm_command = ArmCommand()
        #
        arm_command.type = arm_command.TCP_TYPE
        arm_command.tcp.pos.x = tcp[0]
        arm_command.tcp.pos.y = tcp[1]
        arm_command.tcp.pos.z = tcp[2]

        arm_command.tcp.euler.x = tcp[3]
        arm_command.tcp.euler.y = tcp[4]
        arm_command.tcp.euler.z = tcp[5]
        # 
        self.execute_arm_control(index, arm_command)

    def ArcTo(self,index,mid_tcp, end_tcp):
         # Example commands
        arm_command = ArmCommand()
        #
        arm_command.type = arm_command.TCP_CIRCLE_TYPE
        arm_command.mid_tcp.pos.x = mid_tcp[0]
        arm_command.mid_tcp.pos.y = mid_tcp[1]
        arm_command.mid_tcp.pos.z = mid_tcp[2]
        arm_command.mid_tcp.euler.x = mid_tcp[3]
        arm_command.mid_tcp.euler.y = mid_tcp[4]
        arm_command.mid_tcp.euler.z = mid_tcp[5]
        #
        arm_command.tcp.pos.x = end_tcp[0]
        arm_command.tcp.pos.y = end_tcp[1]
        arm_command.tcp.pos.z = end_tcp[2]
        arm_command.tcp.euler.x = end_tcp[3]
        arm_command.tcp.euler.y = end_tcp[4]
        arm_command.tcp.euler.z = end_tcp[5]
        # 
        self.execute_arm_control(index, arm_command)

    def closeGripper(self, index, force):
        gripper_command = GripperCommand()
        gripper_command.pos = 0
        gripper_command.force = force
        self.execute_gripper_control(index, gripper_command)
    
    def openGripper(self, index,force):
        gripper_command = GripperCommand()
        gripper_command.pos = 1000
        gripper_command.force = force
        self.execute_gripper_control(index, gripper_command)

    def execute_arm_control(self, index, msg):
        """
        Execute arm control for the specified controller index.

        Args:
            index (int): The index of the arm controller.
            msg (grasp_msgs.msg.ArmCommand): The arm command to execute.

        Returns:
            bool: True if the command was successfully executed, False otherwise.
        """
        # Ensure the index is valid
        if index < 0 or index >= len(self.arm_controllers):
            raise IndexError(f"Invalid arm controller index: {index}")

        poses_vector = [msg]
        return self.arm_controllers[index].execute_arm(poses_vector)

    def execute_gripper_control(self, index, msg):
        """
        Execute gripper control for the specified controller index.

        Args:
            index (int): The index of the arm controller.
            msg (grasp_msgs.msg.GripperCommand): The gripper command to execute.

        Returns:
            bool: True if the command was successfully executed, False otherwise.
        """
        # Ensure the index is valid
        if index < 0 or index >= len(self.arm_controllers):
            raise IndexError(f"Invalid arm controller index: {index}")

        return self.arm_controllers[index].execute_gripper(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GraspTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down GraspTaskManager.")
    finally:
        rclpy.shutdown()

# Example usage
if __name__ == "__main__":
    main()
