import math
import torch
import torch.nn as nn

MODEL_PATH = "/home/ubuntu/elderly_ai/data/follow_policy_v3.pth"

model = nn.Sequential(
    nn.Linear(4, 64),
    nn.ReLU(),
    nn.Linear(64, 64),
    nn.ReLU(),
    nn.Linear(64, 2),
)

state = torch.load(MODEL_PATH, map_location="cpu", weights_only=True)
model.load_state_dict(state)
model.eval()

# 各ケース: 距離[m], 目標方向の誤差[rad]
cases = [
    ("正面・距離2m", 2.0, 0.0),
    ("正面・距離0.5m", 0.5, 0.0),
    ("左45度", 2.0, math.pi / 4),
    ("右45度", 2.0, -math.pi / 4),
    ("左90度", 2.0, math.pi / 2),
    ("右90度", 2.0, -math.pi / 2),
    ("背後", 2.0, math.pi),
]

print("モデルの出力確認（指令は発行しません）")
with torch.no_grad():
    for name, distance, heading in cases:
        dx_body = distance * math.cos(heading)
        dy_body = distance * math.sin(heading)
        x = torch.tensor(
            [[dx_body, dy_body, distance, heading]],
            dtype=torch.float32,
        )
        linear_x, angular_z = model(x)[0].tolist()
        print(
            f"{name:14s} "
            f"linear_x={linear_x:+.4f} m/s, "
            f"angular_z={angular_z:+.4f} rad/s"
        )
