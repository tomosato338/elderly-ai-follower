# Elderly AI Follower Robot

Gazebo + ROS2 Humble を用いた高齢者見守りロボットシミュレーション環境です。

## 機能

- Actor歩行シミュレーション
- 差動駆動FollowerBot
- ROS2 cmd_vel制御
- Odom計測
- Behavior Cloning学習
- 学習済みモデル推論

## ディレクトリ構成

data/
  follow_dataset.csv
  follow_policy.pth

ros_nodes/
  follower_controller.py
  train_bc.py
  bc_infer.py

simulation/
  models/
  worlds/

## 動作確認

- follower_bot_new スポーン成功
- cmd_vel前進成功
- cmd_vel旋回成功
- odom取得成功

