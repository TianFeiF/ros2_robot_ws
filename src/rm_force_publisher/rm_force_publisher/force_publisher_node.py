#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml
import os

from rm_ros_interfaces.msg import Sixforce
from geometry_msgs.msg import Pose, WrenchStamped


class ForcePublisher(Node):
    def __init__(self):
        super().__init__('force_publisher')

        # 从 ROS2 参数获取标定文件路径
        self.declare_parameter('calibration_file', '')
        calib_path = self.get_parameter('calibration_file').get_parameter_value().string_value

        if not calib_path or not os.path.exists(calib_path):
            self.get_logger().fatal(f"标定文件不存在: '{calib_path}'，请检查 calibration_file 参数")
            raise RuntimeError("calibration file not found")

        with open(calib_path, 'r') as f:
            calib = yaml.safe_load(f)

        self.mass = calib['mass']
        self.com = np.array(calib['com'])
        self.f_bias = np.array(calib['f_bias'])
        self.m_bias = np.array(calib['m_bias'])
        self.g_world = np.array([0, 0, -9.81])

        self.latest_force = None
        self.latest_rot = None

        self.sub_force = self.create_subscription(
            Sixforce, '/rm_driver/udp_six_force', self.force_cb, 10)
        self.sub_pose = self.create_subscription(
            Pose, '/rm_driver/udp_arm_position', self.pose_cb, 10)
        self.pub_wrench = self.create_publisher(
            WrenchStamped, '/compensated_wrench', 10)

        self.timer = self.create_timer(0.02, self.publish_loop)  # 50Hz
        self.get_logger().info(
            f"force_publisher 已启动，标定文件: {calib_path}，等待数据...")

    def force_cb(self, msg):
        self.latest_force = np.array([
            msg.force_fx, msg.force_fy, msg.force_fz,
            msg.force_mx, msg.force_my, msg.force_mz
        ])

    def pose_cb(self, msg):
        quat = [msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w]
        self.latest_rot = R.from_quat(quat).as_matrix()

    def publish_loop(self):
        if self.latest_force is None or self.latest_rot is None:
            return

        rot = self.latest_rot
        raw = self.latest_force

        # 重力补偿（传感器坐标系下）
        g_local = rot.T @ self.g_world
        f_grav = self.mass * g_local
        f_contact = raw[:3] - f_grav - self.f_bias
        m_grav = np.cross(self.com, f_grav)
        m_contact = raw[3:6] - m_grav - self.m_bias

        # 变换到基座坐标系
        f_base = rot @ f_contact
        m_base = rot @ m_contact

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.wrench.force.x = float(f_base[0])
        msg.wrench.force.y = float(f_base[1])
        msg.wrench.force.z = float(f_base[2])
        msg.wrench.torque.x = float(m_base[0])
        msg.wrench.torque.y = float(m_base[1])
        msg.wrench.torque.z = float(m_base[2])
        self.pub_wrench.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ForcePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
