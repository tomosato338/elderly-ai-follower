#!/usr/bin/env python3
"""体組成・デバイス情報の模擬ノード(シミュレーション専用、ステップ4)。

追従が始まった(/session/state が FOLLOWING になった)とき、
ダミーの値を /session/participant_info に1回だけ出す。
実在の人のデータは使わない。

出力: /session/participant_info (std_msgs/String、JSON 文字列)
  {
    "device_id": "sim-device-01",
    "weight_kg": 52.3,
    "body_fat_percent": 27.4,
    "muscle_mass_kg": 31.8,
    "heart_rate_bpm": 72,
    "sleep_hours": 6.5
  }

実機では、体組成計とスマートデバイスの受信ノードが、同じトピックに
同じ形式で発行すれば、受け取る側(session_recorder.py)は変更不要。
"""

import json
import random

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class ParticipantInfoSim(Node):
    def __init__(self):
        super().__init__('participant_info_sim')

        self.declare_parameter('session_topic', '/session/state')
        self.declare_parameter('info_topic', '/session/participant_info')
        self.declare_parameter('delay', 0.5)   # 追従開始から発行までの遅れ[s]
        self.declare_parameter('device_id', 'sim-device-01')

        p = self.get_parameter
        self.delay = p('delay').value
        self.device_id = p('device_id').value

        self.following = False
        self.timer = None

        self.info_pub = self.create_publisher(
            String, p('info_topic').value, 10
        )
        self.session_sub = self.create_subscription(
            String, p('session_topic').value, self.session_callback, 10
        )

        self.get_logger().info(
            f'Participant info sim started (delay={self.delay} s)'
        )

    def session_callback(self, msg):
        following = msg.data == 'FOLLOWING'

        # IDLE -> FOLLOWING の変化のときだけ、1回発行する
        if following and not self.following:
            if self.timer is not None:
                self.destroy_timer(self.timer)
            self.timer = self.create_timer(self.delay, self.publish_once)
        self.following = following

    def publish_once(self):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None

        info = {
            'device_id': self.device_id,
            'weight_kg': round(random.uniform(40.0, 75.0), 1),
            'body_fat_percent': round(random.uniform(18.0, 35.0), 1),
            'muscle_mass_kg': round(random.uniform(25.0, 40.0), 1),
            'heart_rate_bpm': random.randint(55, 90),
            'sleep_hours': round(random.uniform(4.5, 8.5), 1),
        }
        self.info_pub.publish(String(data=json.dumps(info)))
        # 健康に関わる値は、ログに出さない
        self.get_logger().info('dummy participant info published')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ParticipantInfoSim()
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
