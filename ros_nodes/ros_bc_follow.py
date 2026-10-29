import math

import rclpy
from rclpy.node import Node

import torch
import torch.nn as nn

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


class BcFollowNode(Node):
    def __init__(self):
        super().__init__("bc_follow_node")

        # ===== 設定 =====
        self.model_path = (
            "/home/ubuntu/elderly_ai/data/follow_policy.pth"
        )
        self.model_states_topic = "/model_states"

        self.robot_name = "follower_bot"
        self.actor_name = "actor"

        # 安全のための速度上限（環境に合わせて調整）
        self.max_linear_speed = 0.3  # m/s
        self.max_angular_speed = 0.0  # rad/s

        # 座標が古くなったら制御を停止する
        self.position_timeout_sec = 1.0

        self.robot_position = None
        self.actor_position = None
        self.last_position_time = None

        # ===== AIモデルの準備 =====
        self.model = nn.Sequential(
            nn.Linear(4, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
        )

        try:
            state_dict = torch.load(
                self.model_path,
                map_location="cpu",
                weights_only=True,
            )
            self.model.load_state_dict(state_dict)
            self.model.eval()
        except Exception as exc:
            raise RuntimeError(
                f"AIモデルの読み込みに失敗しました: {exc}"
            ) from exc

        self.get_logger().info("AI model loaded successfully.")

        # ===== 移動指令 =====
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        # ===== 座標の購読 =====
        self.position_sub = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.position_callback,
            10,
        )

        # 5 Hzで制御
        self.timer = self.create_timer(
            0.2,
            self.control_loop,
        )

        self.get_logger().info(
            f"Subscribing to {self.model_states_topic}"
        )
        self.get_logger().info(
            "BC follow controller initialized."
        )

    def position_callback(self, msg):
        """ModelStatesメッセージから対象モデルの座標を取得する。"""
        try:
            if self.robot_name not in msg.name:
                self.get_logger().warning(
                    f"Robot model '{self.robot_name}' not found."
                )
                self.robot_position = None
                return

            if self.actor_name not in msg.name:
                self.get_logger().warning(
                    f"Actor model '{self.actor_name}' not found."
                )
                self.actor_position = None
                return

            robot_idx = msg.name.index(self.robot_name)
            actor_idx = msg.name.index(self.actor_name)

            robot_pose = msg.pose[robot_idx].position
            actor_pose = msg.pose[actor_idx].position

            values = [
                robot_pose.x,
                robot_pose.y,
                actor_pose.x,
                actor_pose.y,
            ]

            if not all(math.isfinite(v) for v in values):
                self.get_logger().warning(
                    "Non-finite position received."
                )
                self.robot_position = None
                self.actor_position = None
                return

            self.robot_position = (
                float(robot_pose.x),
                float(robot_pose.y),
            )
            self.actor_position = (
                float(actor_pose.x),
                float(actor_pose.y),
            )
            self.last_position_time = self.get_clock().now()

        except (IndexError, ValueError) as exc:
            self.get_logger().error(
                f"Failed to parse model positions: {exc}"
            )
            self.robot_position = None
            self.actor_position = None

    def publish_stop(self):
        """ゼロ速度指令を送信する。"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

    def positions_are_fresh(self):
        """座標が存在し、かつ古くないことを確認する。"""
        if (
            self.robot_position is None
            or self.actor_position is None
            or self.last_position_time is None
        ):
            return False

        elapsed = (
            self.get_clock().now() - self.last_position_time
        ).nanoseconds / 1e9

        return 0.0 <= elapsed <= self.position_timeout_sec

    def control_loop(self):
        """座標を確認し、条件を満たす場合のみAI制御を実行する。"""
        if not self.positions_are_fresh():
            self.publish_stop()
            self.get_logger().warning(
                "Position data unavailable or stale; stop command sent."
            )
            return

        robot_x, robot_y = self.robot_position
        actor_x, actor_y = self.actor_position

        # 学習時と同じ入力順序を使用すること
        state = torch.tensor(
            [[actor_x, actor_y, robot_x, robot_y]],
            dtype=torch.float32,
        )

        try:
            with torch.no_grad():
                linear_x = float(self.model(state).item())

            if not math.isfinite(linear_x):
                self.get_logger().error(
                    "AI output is not finite; stopping."
                )
                self.publish_stop()
                return

            # 速度を制限する
            linear_x = max(
                -self.max_linear_speed,
                min(self.max_linear_speed, linear_x),
            )

            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = self.max_angular_speed
            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"Actor=({actor_x:.2f},{actor_y:.2f}) "
                f"Robot=({robot_x:.2f},{robot_y:.2f}) "
                f"V={linear_x:.2f}"
            )

        except Exception as exc:
            self.get_logger().error(
                f"Control error: {exc}"
            )
            self.publish_stop()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = BcFollowNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        print(f"FATAL ERROR: {exc}")

    finally:
        if node is not None:
            # 終了時にも停止指令を送る
            try:
                node.publish_stop()
            except Exception:
                pass

            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
