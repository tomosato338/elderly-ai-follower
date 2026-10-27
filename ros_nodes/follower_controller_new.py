#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller_new')

        self.actor_name = 'actor'
        self.robot_name = 'follower_bot_new'

        # 追従設定
        self.desired_distance = 0.8
        self.max_linear_speed = 0.40
        self.max_angular_speed = 1.5
        self.linear_gain = 0.60
        self.angular_gain = 1.2

        # 旋回中の前進速度制御
        self.forward_angle_limit = 0.45
        self.minimum_alignment = 0.40

        self.last_states_time = 0.0
        self.robot_pose = None
        self.actor_pose = None

        self.cmd_pub = self.create_publisher(
            Twist, '/follower_bot_new/cmd_vel', 10
        )
        self.states_sub = self.create_subscription(
            ModelStates, '/model_states',
            self.model_states_callback, 10
        )
        self.control_timer = self.create_timer(
            0.1, self.control_loop
        )

        self.get_logger().info(
            'Follower controller started: '
            'max_linear_speed=0.40 m/s'
        )

    def model_states_callback(self, msg):
        self.last_states_time = time.monotonic()

        try:
            actor_index = msg.name.index(self.actor_name)
            robot_index = msg.name.index(self.robot_name)
        except ValueError:
            self.actor_pose = None
            self.robot_pose = None
            return

        actor = msg.pose[actor_index]
        robot = msg.pose[robot_index]

        self.actor_pose = (
            actor.position.x,
            actor.position.y,
        )

        q = robot.orientation
        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(sin_yaw, cos_yaw)

        self.robot_pose = (
            robot.position.x,
            robot.position.y,
            yaw,
        )

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if (
            self.actor_pose is None
            or self.robot_pose is None
            or time.monotonic() - self.last_states_time > 1.0
        ):
            self.publish_stop()
            return

        rx, ry, yaw = self.robot_pose
        ax, ay = self.actor_pose

        dx = ax - rx
        dy = ay - ry
        distance = math.hypot(dx, dy)

        target_heading = math.atan2(dy, dx)
        heading_error = math.atan2(
            math.sin(target_heading - yaw),
            math.cos(target_heading - yaw),
        )

        cmd = Twist()

        # 距離にかかわらず、まず目標方向への旋回を計算
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                self.angular_gain * heading_error,
            ),
        )

        # 目標距離以内では前進しない。ただし旋回は継続する
        if distance <= self.desired_distance:
            cmd.linear.x = 0.0

        # 向きが十分に合うまでは前進しない
        elif abs(heading_error) < math.pi / 2:
            distance_speed = self.linear_gain * (distance - self.desired_distance)
            alignment = max(self.minimum_alignment, math.cos(heading_error))
            cmd.linear.x = min(self.max_linear_speed, distance_speed) * alignment
        else:
            cmd.linear.x = 0.0


        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'distance={distance:.2f} m, '
            f'heading_error={math.degrees(heading_error):.1f} deg, '
            f'linear={cmd.linear.x:.2f} m/s, '
            f'angular={cmd.angular.z:.2f} rad/s'
        )

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FollowerController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止指令を数回送って、確実に届ける
        try:
            for _ in range(3):
                node.publish_stop()
                time.sleep(0.05)
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
