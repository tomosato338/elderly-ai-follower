import csv
import math
import random

OUTPUT = "/home/ubuntu/elderly_ai/data/follow_dataset_v3.csv"
SAMPLES = 20000
DT = 0.05

MAX_LINEAR = 0.3
MAX_ANGULAR = 0.8
DESIRED_DISTANCE = 1.0

random.seed(42)

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

with open(OUTPUT, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "dx_body", "dy_body", "distance",
        "heading_error", "linear_x", "angular_z"
    ])

    for _ in range(SAMPLES):
        # ロボットの姿勢をランダム化
        robot_x = random.uniform(-2.0, 2.0)
        robot_y = random.uniform(-2.0, 2.0)
        robot_yaw = random.uniform(-math.pi, math.pi)

        # 目標との距離・方向を幅広くサンプリング
        distance = random.uniform(0.3, 3.0)
        heading_error = random.uniform(-math.pi, math.pi)

        # 相対状態から目標のワールド座標を作る
        target_heading = robot_yaw + heading_error
        actor_x = robot_x + distance * math.cos(target_heading)
        actor_y = robot_y + distance * math.sin(target_heading)

        dx = actor_x - robot_x
        dy = actor_y - robot_y

        dx_body = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
        dy_body = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy

        # 目標の方向へ旋回する教師制御
        angular_z = max(
            -MAX_ANGULAR,
            min(MAX_ANGULAR, 1.5 * heading_error)
        )

        # 目標が後方や大きく横にいる場合は旋回を優先
        if abs(heading_error) > 0.5:
            linear_x = 0.0
        else:
            linear_x = max(
                -MAX_LINEAR,
                min(MAX_LINEAR, 0.5 * (distance - DESIRED_DISTANCE))
            )

        writer.writerow([
            dx_body, dy_body, distance,
            heading_error, linear_x, angular_z
        ])

print(f"Saved {SAMPLES} samples to {OUTPUT}")
