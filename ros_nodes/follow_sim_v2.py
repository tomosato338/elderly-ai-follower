import csv
import math
import random

OUTPUT = "/home/ubuntu/elderly_ai/data/follow_dataset_v2.csv"

DT = 0.05
SAMPLES = 20000

# 教師制御用の上限
MAX_LINEAR = 0.3
MAX_ANGULAR = 0.8

# 目標との希望距離
DESIRED_DISTANCE = 1.0

# 再現可能なデータ生成
random.seed(42)

def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

# 目標とロボットの初期状態
actor_x = 1.0
actor_y = 0.0
robot_x = -1.0
robot_y = 0.0
robot_yaw = 0.0

with open(OUTPUT, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "dx_body",
        "dy_body",
        "distance",
        "heading_error",
        "linear_x",
        "angular_z",
    ])

    for step in range(SAMPLES):
        t = step * DT

        # 目標は半径1.5mの円周上を移動
        actor_x = 1.5 * math.cos(0.25 * t)
        actor_y = 1.5 * math.sin(0.25 * t)

        dx = actor_x - robot_x
        dy = actor_y - robot_y
        distance = math.hypot(dx, dy)

        # 目標方向をロボット座標系へ変換
        dx_body = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
        dy_body = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy

        heading_error = wrap_angle(math.atan2(dy, dx) - robot_yaw)

        # 教師となる単純な追従制御
        angular_z = max(
            -MAX_ANGULAR,
            min(MAX_ANGULAR, 1.5 * heading_error)
        )

        if distance > DESIRED_DISTANCE:
            linear_x = min(MAX_LINEAR, 0.5 * (distance - DESIRED_DISTANCE))
        else:
            linear_x = 0.0

        # 目標が大きく横または後方にいる場合は旋回を優先
        if abs(heading_error) > 0.7:
            linear_x = 0.0

        writer.writerow([
            dx_body,
            dy_body,
            distance,
            heading_error,
            linear_x,
            angular_z,
        ])

        # 差動二輪ロボットの簡易運動モデル
        robot_x += linear_x * math.cos(robot_yaw) * DT
        robot_y += linear_x * math.sin(robot_yaw) * DT
        robot_yaw = wrap_angle(robot_yaw + angular_z * DT)

print(f"Saved {SAMPLES} samples to {OUTPUT}")
