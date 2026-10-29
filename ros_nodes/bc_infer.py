import torch
import torch.nn as nn

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

state = torch.tensor(
    [[
        1.0,   # actor_x
        0.0,   # actor_y
        2.0,   # robot_x
        0.0    # robot_y
    ]],
    dtype=torch.float32
)

with torch.no_grad():

    action = model(state)

print("predicted linear_x =", action.item())
