import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import torch
import torch.nn as nn


MODEL_PATH = "/home/ubuntu/elderly_ai/data/follow_policy_v3.pth"


class FollowDryRun(Node):
    def __init__(self):
        super().__init__("follow_v3_dryrun")

        self.model = nn.Sequential(
            nn.Linear(4, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 2),
        )
        state = torch.load(
            MODEL_PATH, map_location="cpu", weights_only=True
        )
        self.model.load_state_dict(state)
        self.model.eval()

        self.latest = None

        self.sub = self.create_subscription(
            ModelStates,
            "/model_states",
            self.model_states_callback,
            10,
        )
        self.timer = self.create_timer(1.0, self.report)

        self.get_logger().info(
            "Dry-run開始: /model_statesを読み取ります。"
            "/cmd_velにはpublishしません。"
        )

    def model_states_callback(self, msg):
        try:
            actor_i = msg.name.index("actor")
            robot_i = msg.name.index("follower_bot")
        except ValueError:
            return

        actor = msg.pose[actor_i]
        robot = msg.pose[robot_i]

        q = robot.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        dx = actor.position.x - robot.position.x
        dy = actor.position.y - robot.position.y

        dx_body = math.cos(yaw) * dx + math.sin(yaw) * dy
        dy_body = -math.sin(yaw) * dx + math.cos(yaw) * dy
        distance = math.hypot(dx, dy)
        heading_error = math.atan2(dy_body, dx_body)

        self.latest = (
            dx_body, dy_body, distance, heading_error, yaw
        )

    def report(self):
        if self.latest is None:
            self.get_logger().warn(
                "actor / follower_bot の姿勢データを待っています"
            )
            return

        dx_body, dy_body, distance, heading_error, yaw = self.latest
        x = torch.tensor(
            [[dx_body, dy_body, distance, heading_error]],
            dtype=torch.float32,
        )

        with torch.no_grad():
            linear_x, angular_z = self.model(x)[0].tolist()

        self.get_logger().info(
            f"distance={distance:.3f}m, "
            f"heading_error={heading_error:+.3f}rad, "
            f"yaw={yaw:+.3f}rad | "
            f"予測 linear_x={linear_x:+.3f}m/s, "
            f"angular_z={angular_z:+.3f}rad/s "
            "(表示のみ・指令送信なし)"
        )


def main():
    rclpy.init()
    node = FollowDryRun()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
