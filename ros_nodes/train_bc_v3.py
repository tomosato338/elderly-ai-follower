import random
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

DATA_PATH = "/home/ubuntu/elderly_ai/data/follow_dataset_v3.csv"
MODEL_PATH = "/home/ubuntu/elderly_ai/data/follow_policy_v3.pth"

SEED = 42
BATCH_SIZE = 256
EPOCHS = 100
LEARNING_RATE = 1e-3

random.seed(SEED)
np.random.seed(SEED)
torch.manual_seed(SEED)

df = pd.read_csv(DATA_PATH)

input_cols = [
    "dx_body",
    "dy_body",
    "distance",
    "heading_error",
]
target_cols = ["linear_x", "angular_z"]

X = df[input_cols].to_numpy(dtype=np.float32)
y = df[target_cols].to_numpy(dtype=np.float32)

if not np.isfinite(X).all() or not np.isfinite(y).all():
    raise ValueError("データにNaNまたは無限大があります")

# ランダム分割して学習用・検証用を分ける
indices = np.random.permutation(len(df))
split = int(len(df) * 0.8)
train_idx, val_idx = indices[:split], indices[split:]

X_train = torch.from_numpy(X[train_idx])
y_train = torch.from_numpy(y[train_idx])
X_val = torch.from_numpy(X[val_idx])
y_val = torch.from_numpy(y[val_idx])

train_loader = DataLoader(
    TensorDataset(X_train, y_train),
    batch_size=BATCH_SIZE,
    shuffle=True,
)

model = nn.Sequential(
    nn.Linear(4, 64),
    nn.ReLU(),
    nn.Linear(64, 64),
    nn.ReLU(),
    nn.Linear(64, 2),
)

optimizer = torch.optim.Adam(
    model.parameters(),
    lr=LEARNING_RATE,
)
loss_fn = nn.MSELoss()

best_val_loss = float("inf")
best_state = None

for epoch in range(1, EPOCHS + 1):
    model.train()
    train_total = 0.0

    for xb, yb in train_loader:
        pred = model(xb)
        loss = loss_fn(pred, yb)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        train_total += loss.item() * len(xb)

    train_loss = train_total / len(train_idx)

    model.eval()
    with torch.no_grad():
        val_pred = model(X_val)
        val_loss = loss_fn(val_pred, y_val).item()

    if val_loss < best_val_loss:
        best_val_loss = val_loss
        best_state = {
            k: v.detach().cpu().clone()
            for k, v in model.state_dict().items()
        }

    if epoch == 1 or epoch % 10 == 0:
        print(
            f"Epoch {epoch:3d}/{EPOCHS} "
            f"train_MSE={train_loss:.6f} "
            f"val_MSE={val_loss:.6f}"
        )

model.load_state_dict(best_state)
model.eval()

with torch.no_grad():
    pred = model(X_val).numpy()

actual = y[val_idx]
mae = np.mean(np.abs(pred - actual), axis=0)
rmse = np.sqrt(np.mean((pred - actual) ** 2, axis=0))

print("\n検証結果（ランダム分割）")
print(f"検証件数: {len(val_idx)}")
print(f"linear_x MAE:  {mae[0]:.6f} m/s")
print(f"linear_x RMSE: {rmse[0]:.6f} m/s")
print(f"angular_z MAE:  {mae[1]:.6f} rad/s")
print(f"angular_z RMSE: {rmse[1]:.6f} rad/s")

torch.save(best_state, MODEL_PATH)
print(f"\n保存しました: {MODEL_PATH}")
