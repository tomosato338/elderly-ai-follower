#!/usr/bin/env bash
# Gazebo(actor_physics_test.world)を起動し、follower_bot_new を生成する。
#
# 使い方:
#   ./start_sim.sh          (Ctrl+C で Gazebo ごと終了)
#
# 環境変数で上書きできる:
#   WORLD    ワールドファイルのパス
#   SPAWN_X, SPAWN_Y, SPAWN_Z   ロボットの生成位置[m]
#            (actor から 1.5 m 以上離すこと)

ROOT="$(cd "$(dirname "$0")" && pwd)"
WORLD="${WORLD:-$ROOT/simulation/worlds/actor_physics_test.world}"
MODEL="$ROOT/simulation/models/follower_bot_new/model.sdf"
SPAWN_X="${SPAWN_X:-2.0}"
SPAWN_Y="${SPAWN_Y:-0.0}"
SPAWN_Z="${SPAWN_Z:-0.0}"

source /opt/ros/humble/setup.bash

for f in "$WORLD" "$MODEL"; do
  if [ ! -f "$f" ]; then
    echo "ファイルが見つかりません: $f" >&2
    exit 1
  fi
done

# /spawn_entity が使えるようになるまで待ってから、ロボットを生成する
(
  ros2 run gazebo_ros spawn_entity.py \
    -entity follower_bot_new \
    -file "$MODEL" \
    -x "$SPAWN_X" -y "$SPAWN_Y" -z "$SPAWN_Z" \
    -timeout 120
) &

# Gazebo は前面で実行する(Ctrl+C で終了できる)
gazebo "$WORLD" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so
