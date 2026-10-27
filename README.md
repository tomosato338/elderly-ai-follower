# Elderly AI Follower Robot

Development of a Follow-Along Simulator for Elderly Care Robots.

Gazebo + ROS 2 Humble を用いた高齢者見守りロボットシミュレーション環境です。

## 機能

- Actor歩行シミュレーション
- 差動駆動FollowerBot
- ROS 2 cmd_vel制御
- Odom計測
- Behavior Cloning学習
- 学習済みモデル推論

## ディレクトリ構成

```text
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
