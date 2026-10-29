import torch
import torch.nn as nn
import math
import time

model = nn.Sequential(
    nn.Linear(4,64),
    nn.ReLU(),
    nn.Linear(64,64),
    nn.ReLU(),
    nn.Linear(64,1)
)

model.load_state_dict(
    torch.load(
        "/home/ubuntu/elderly_ai/data/follow_policy.pth"
    )
)

model.eval()

robot_x = 2.0
robot_y = 0.0

t = 0.0

while True:

    actor_x = math.cos(t)
    actor_y = math.sin(t)

    state = torch.tensor(
        [[
            actor_x,
            actor_y,
            robot_x,
            robot_y
        ]],
        dtype=torch.float32
    )

    with torch.no_grad():
        linear_x = (
            model(state)
            .item()
        )

    heading = math.atan2(
        actor_y - robot_y,
        actor_x - robot_x
    )

    dt = 0.05

    robot_x += (
        linear_x *
        math.cos(heading) *
        dt
    )

    robot_y += (
        linear_x *
        math.sin(heading) *
        dt
    )

    print(
        f"Actor({actor_x:.2f},{actor_y:.2f}) "
        f"Robot({robot_x:.2f},{robot_y:.2f}) "
        f"V={linear_x:.2f}"
    )

    t += 0.05

    time.sleep(0.05)

