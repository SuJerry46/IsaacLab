import threading

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointSubPub(Node):
    def __init__(self):
        super().__init__("ros2_isaac_joint_reader")

        self.lock = threading.Lock()
        self.name = None
        self.position = None
        self.velocity = None
        self.effort = None

        self.sub = self.create_subscription(
            JointState, "isaac_sim/command_joint_states", self.callback, 1
        )
        self.pub = self.create_publisher(JointState, "isaac_sim/joint_states", 1)

    def callback(self, msg):
        with self.lock:
            self.name = np.array(msg.name)
            self.position = np.array(msg.position)
            self.velocity = np.array(msg.velocity)
            self.effort = np.array(msg.effort)

    def get_joint_name(self):
        if self.name is None:
            self.get_logger().warn("There is no joint_name data available!")
            return

        with self.lock:
            return self.name

    def get_joint_position(self):
        if self.position is None:
            self.get_logger().warn("There is no joint_position data available!")
            return

        with self.lock:
            return self.position

    def get_joint_velocity(self):
        if self.velocity is None:
            self.get_logger().warn("There is no joint_velocity data available!")
            return

        with self.lock:
            return self.velocity

    def get_joint_effort(self):
        if self.effort is None:
            self.get_logger().warn("There is no joint_effort data available!")
            return

        with self.lock:
            return self.effort

    def pub_current_joint(
        self, joint_name, joint_position, joint_velocity=None, joint_effort=None
    ):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = joint_name
        if joint_position.ndim > 1:
            joint_position = joint_position.flatten()
        if joint_velocity.ndim > 1:
            joint_velocity = joint_velocity.flatten()
        if joint_effort.ndim > 1:
            joint_effort = joint_effort.flatten()

        msg.position = [float(x) for x in joint_position]
        msg.velocity = [float(x) for x in joint_velocity]
        msg.effort = [float(x) for x in joint_effort]

        self.pub.publish(msg)


class Ros2Interface:
    def __init__(self):
        self.node = None
        self.spin_thread = None

    def start_ros2_spin_thread(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = JointSubPub()

        def spin():
            try:
                rclpy.spin(self.node)
            except Exception:
                self.node.destroy_node()
                rclpy.shutdown()

        self.spin_thread = threading.Thread(target=spin, daemon=True)
        self.spin_thread.start()

    def stop_ros2_spin_thread(self):
        if self.node is not None:
            self.node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        if self.spin_thread is not None and self.spin_thread.is_alive():
            self.spin_thread.join()

        self.node = None

    def is_running(self):
        if self.node is None:
            return False
        else:
            return True
