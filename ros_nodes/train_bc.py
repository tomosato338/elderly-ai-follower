import pandas as pd
import torch
import torch.nn as nn

# CSV読込
df = pd.read_csv(
    "/home/ubuntu/elderly_ai/data/follow_dataset.csv"
)

X = df[
    [
        "actor_x",
        "actor_y",
        "robot_x",
        "robot_y"
    ]
].values

y = df[
    [
        "linear_x"
    ]
].values

X = torch.tensor(
    X,
    dtype=torch.float32
)

y = torch.tensor(
    y,
    dtype=torch.float32
)

model = nn.Sequential(
    nn.Linear(4,64),
    nn.ReLU(),
    nn.Linear(64,64),
    nn.ReLU(),
    nn.Linear(64,1)
)

optimizer = torch.optim.Adam(
    model.parameters(),
    lr=1e-3
)

loss_fn = nn.MSELoss()

for epoch in range(1000):

    pred = model(X)

    loss = loss_fn(
        pred,
        y
    )

    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    if epoch % 100 == 0:
        print(
            epoch,
            loss.item()
        )

torch.save(
    model.state_dict(),
    "/home/ubuntu/elderly_ai/data/follow_policy.pth"
)

print("saved")
