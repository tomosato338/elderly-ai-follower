#!/usr/bin/env python3
"""セッション管理ノード(ステップ2)。

状態: IDLE(待機) / FOLLOWING(追従中)

入力:
  /device/event (std_msgs/String)
      'connect' : デバイスの接続。デバイスが connect_distance 以内に
                  あるときだけ受け付け、FOLLOWING に移る
      'reset'   : リセットボタン。停止指令を送り、IDLE に戻る
  /device/relative_position (geometry_msgs/PointStamped)
      接続時の近接判定に使う(ロボット座標系での相対位置[m])

出力:
  /session/state (std_msgs/String)
      現在の状態。変化したときと、1秒ごとに発行する
  /follower_bot_new/cmd_vel (geometry_msgs/Twist)
      リセット時と終了時の停止指令

動作確認(シミュレーション):
  ros2 topic pub -r 2 -t 3 /device/event std_msgs/msg/String "{data: connect}"
  ros2 topic pub -r 2 -t 3 /device/event std_msgs/msg/String "{data: reset}"
  (同じイベントを3回送っても、2回目以降は無視されるので問題ない)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg import String

IDLE = 'IDLE'
FOLLOWING = 'FOLLOWING'


class SessionManager(Node):
    def __init__(self):
        super().__init__('session_manager')

        self.declare_parameter('connect_distance', 1.0)   # [m]
        self.declare_parameter('signal_timeout', 1.0)     # [s]
        self.declare_parameter('event_topic', '/device/event')
        self.declare_parameter('state_topic', '/session/state')
        self.declare_parameter('signal_topic', '/device/relative_position')
        self.declare_parameter('cmd_vel_topic', '/follower_bot_new/cmd_vel')

        p = self.get_parameter
        self.connect_distance = p('connect_distance').value
        self.signal_timeout = p('signal_timeout').value

        self.state = IDLE
        self.last_position = None      # (x, y) ロボット座標系
        self.last_signal_time = 0.0

        self.state_pub = self.create_publisher(
            String, p('state_topic').value, 10
        )
        self.cmd_pub = self.create_publisher(
            Twist, p('cmd_vel_topic').value, 10
        )
        self.signal_sub = self.create_subscription(
            PointStamped, p('signal_topic').value,
            self.signal_callback, 10
        )
        self.event_sub = self.create_subscription(
            String, p('event_topic').value, self.event_callback, 10
        )
        # 状態を1秒ごとに発行する(制御ノードは、状態が途切れたら停止する)
        self.state_timer = self.create_timer(1.0, self.publish_state)

        self.publish_state()
        self.get_logger().info(
            f'Session manager started: state={self.state}, '
            f'connect_distance={self.connect_distance} m'
        )

    # ---- 入力 ----
    def signal_callback(self, msg):
        self.last_signal_time = time.monotonic()
        self.last_position = (msg.point.x, msg.point.y)

    def event_callback(self, msg):
        event = msg.data.strip().lower()
        if event == 'connect':
            self.handle_connect()
        elif event == 'reset':
            self.handle_reset()
        else:
            self.get_logger().warn(f'unknown event: "{msg.data}"')

    # ---- イベント処理 ----
    def handle_connect(self):
        if self.state == FOLLOWING:
            self.get_logger().info('already following; connect ignored')
            return

        age = time.monotonic() - self.last_signal_time
        if self.last_position is None or age > self.signal_timeout:
            self.get_logger().warn('connect rejected: no device signal')
            return

        x, y = self.last_position
        distance = math.hypot(x, y)
        if distance > self.connect_distance:
            self.get_logger().warn(
                f'connect rejected: device is {distance:.2f} m away '
                f'(limit {self.connect_distance} m)'
            )
            return

        self.get_logger().info(
            f'connect accepted (device at {distance:.2f} m)'
        )
        self.set_state(FOLLOWING)

    def handle_reset(self):
        # diff_drive プラグインは最後の指令を保持するため、必ず停止を送る
        self.send_stop()
        if self.state == FOLLOWING:
            self.get_logger().info('reset: follow ended')
            self.set_state(IDLE)
        else:
            self.get_logger().info('reset received (already IDLE)')

    # ---- 状態と出力 ----
    def set_state(self, new_state):
        if new_state != self.state:
            self.get_logger().info(f'state: {self.state} -> {new_state}')
            self.state = new_state
        self.publish_state()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def send_stop(self):
        for _ in range(3):
            self.cmd_pub.publish(Twist())

    def shutdown_session(self):
        """終了時: 待機に戻し、停止指令を送る。"""
        self.state = IDLE
        self.publish_state()
        self.send_stop()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SessionManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_session()
            time.sleep(0.1)
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
