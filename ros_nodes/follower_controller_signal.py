#!/usr/bin/env python3
"""デバイス信号に追従する制御ノード(ステップ2: セッション状態に対応)。

入力: /device/relative_position (geometry_msgs/PointStamped)
      ロボット座標系(x=前方, y=左)でのデバイスの相対位置[m]
      /session/state (std_msgs/String)
      'FOLLOWING' のときだけ追従する。それ以外は停止する。
出力: /follower_bot_new/cmd_vel (geometry_msgs/Twist)

追従の計算は、ステップ1と同じ。
追加点:
  - セッション状態が FOLLOWING でなければ停止する
  - セッション状態が state_timeout 秒以上更新されなければ停止する
    (セッション管理ノードが止まったときの安全策)
  - パラメータ require_session:=false で、ステップ1と同じ動作に戻せる
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String


class FollowerControllerSignal(Node):
    def __init__(self):
        super().__init__('follower_controller_signal')

        # パラメータ(起動時に --ros-args -p で変更できる)
        self.declare_parameter('signal_topic', '/device/relative_position')
        self.declare_parameter('cmd_vel_topic', '/follower_bot_new/cmd_vel')
        self.declare_parameter('session_topic', '/session/state')
        self.declare_parameter('require_session', True)
        self.declare_parameter('state_timeout', 3.0)  # s
        self.declare_parameter('desired_distance', 0.8)
        self.declare_parameter('max_linear_speed', 0.40)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('linear_gain', 0.60)
        self.declare_parameter('angular_gain', 1.2)
        self.declare_parameter('forward_angle_limit', 0.45)  # rad
        self.declare_parameter('signal_timeout', 1.0)  # s

        p = self.get_parameter
        self.require_session = p('require_session').value
        self.state_timeout = p('state_timeout').value
        self.desired_distance = p('desired_distance').value
        self.max_linear_speed = p('max_linear_speed').value
        self.max_angular_speed = p('max_angular_speed').value
        self.linear_gain = p('linear_gain').value
        self.angular_gain = p('angular_gain').value
        self.forward_angle_limit = p('forward_angle_limit').value
        self.signal_timeout = p('signal_timeout').value

        self.target = None            # (x, y) ロボット座標系
        self.last_signal_time = 0.0
        self.session_state = 'IDLE'
        self.last_state_time = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, p('cmd_vel_topic').value, 10
        )
        self.signal_sub = self.create_subscription(
            PointStamped, p('signal_topic').value,
            self.signal_callback, 10
        )
        self.session_sub = self.create_subscription(
            String, p('session_topic').value,
            self.session_callback, 10
        )
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Follower (signal) started: '
            f'require_session={self.require_session}, '
            f'desired_distance={self.desired_distance} m, '
            f'max_linear_speed={self.max_linear_speed} m/s, '
            f'max_angular_speed={self.max_angular_speed} rad/s'
        )

    def signal_callback(self, msg):
        self.last_signal_time = time.monotonic()
        self.target = (msg.point.x, msg.point.y)

    def session_callback(self, msg):
        self.last_state_time = time.monotonic()
        if msg.data != self.session_state:
            self.get_logger().info(
                f'session state: {self.session_state} -> {msg.data}'
            )
        self.session_state = msg.data

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        now = time.monotonic()

        # セッションが FOLLOWING でなければ停止
        if self.require_session:
            state_age = now - self.last_state_time
            if state_age > self.state_timeout:
                self.get_logger().warn(
                    'session state not received; stopped',
                    throttle_duration_sec=5.0,
                )
                self.publish_stop()
                return
            if self.session_state != 'FOLLOWING':
                self.get_logger().info(
                    f'idle (session={self.session_state})',
                    throttle_duration_sec=5.0,
                )
                self.publish_stop()
                return

        age = now - self.last_signal_time

        # 信号が来ていない、または古い場合は停止
        if self.target is None or age > self.signal_timeout:
            self.get_logger().warn(
                f'waiting for signal (age={age:.1f}s)',
                throttle_duration_sec=2.0,
            )
            self.publish_stop()
            return

        x, y = self.target
        distance = math.hypot(x, y)
        # ロボット座標系なので、方位誤差は相対位置の方位角そのもの
        heading_error = math.atan2(y, x)

        cmd = Twist()

        # 旋回(距離にかかわらず計算し、上限で制限)
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, self.angular_gain * heading_error),
        )

        # 前進
        if distance <= self.desired_distance:
            cmd.linear.x = 0.0
        elif abs(heading_error) <= self.forward_angle_limit:
            distance_speed = self.linear_gain * (
                distance - self.desired_distance
            )
            cmd.linear.x = min(
                self.max_linear_speed, distance_speed
            ) * max(0.0, math.cos(heading_error))
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'distance={distance:.2f} m, '
            f'heading_error={math.degrees(heading_error):.1f} deg, '
            f'linear={cmd.linear.x:.2f} m/s, '
            f'angular={cmd.angular.z:.2f} rad/s',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    # SIGINT を rclpy に任せず、KeyboardInterrupt として受ける。
    # これにより、Ctrl+C のときも停止指令を確実に送れる。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = FollowerControllerSignal()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # diff_drive プラグインは最後の指令を保持し続けるため、
        # 停止指令(速度0)を数回送って確実に届ける
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
