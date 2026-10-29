#!/usr/bin/env python3
"""デバイス信号の模擬ノード(シミュレーション専用)。

/model_states から actor とロボットの位置を読み、
ロボットから見た actor(=デバイス)の相対位置を発行する。

出力: /device/relative_position (geometry_msgs/PointStamped)
      frame_id='base_link'、point.x=前方[m]、point.y=左[m]

実機では、UWB など実デバイスの受信ノードが、同じトピックに
同じ形式で発行すれば、制御ノードは変更せずに使える。
"""

import math
import random

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped


class DeviceSignalSim(Node):
    def __init__(self):
        super().__init__('device_signal_sim')

        self.declare_parameter('actor_name', 'actor')
        self.declare_parameter('robot_name', 'follower_bot_new')
        self.declare_parameter('signal_topic', '/device/relative_position')
        self.declare_parameter('noise_std', 0.0)      # [m] 位置の雑音
        self.declare_parameter('dropout_prob', 0.0)   # 0.0〜1.0 欠測確率
        self.declare_parameter('max_range', 5.0)      # [m] 届く距離の上限

        p = self.get_parameter
        self.actor_name = p('actor_name').value
        self.robot_name = p('robot_name').value
        self.noise_std = p('noise_std').value
        self.dropout_prob = p('dropout_prob').value
        self.max_range = p('max_range').value

        self.pub = self.create_publisher(
            PointStamped, p('signal_topic').value, 10
        )
        self.sub = self.create_subscription(
            ModelStates, '/model_states', self.callback, 10
        )

        self.get_logger().info(
            f'Device signal sim started: actor={self.actor_name}, '
            f'noise_std={self.noise_std} m, '
            f'dropout_prob={self.dropout_prob}, '
            f'max_range={self.max_range} m'
        )

    def callback(self, msg):
        try:
            actor = msg.pose[msg.name.index(self.actor_name)]
            robot = msg.pose[msg.name.index(self.robot_name)]
        except ValueError:
            self.get_logger().warn(
                f'name not found. available={list(msg.name)}',
                throttle_duration_sec=2.0,
            )
            return

        # ロボットの向き(yaw)
        q = robot.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        # 世界座標の差を、ロボット座標系(x=前方, y=左)に変換
        dx = actor.position.x - robot.position.x
        dy = actor.position.y - robot.position.y
        x = math.cos(yaw) * dx + math.sin(yaw) * dy
        y = -math.sin(yaw) * dx + math.cos(yaw) * dy

        # 届く距離を超えたら、信号なし(何も発行しない)
        if math.hypot(x, y) > self.max_range:
            return

        # 欠測の模擬
        if random.random() < self.dropout_prob:
            return

        # 雑音の模擬
        if self.noise_std > 0.0:
            x += random.gauss(0.0, self.noise_std)
            y += random.gauss(0.0, self.noise_std)

        out = PointStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.point.x = x
        out.point.y = y
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DeviceSignalSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
