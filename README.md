# Elderly AI Follower Robot

Gazebo + ROS 2 Humble を用いた、高齢者見守りロボットの追従シミュレーションです。

ロボット `follower_bot_new` が、接続したデバイス(シミュレーションでは actor)の信号を目標距離 0.8 m で追従し、追従中のカメラ映像と体組成・デバイス情報を、セッションごとに記録します。リセットすると、そのセッションの記録を削除します。

## 主な機能

- デバイスの相対位置(距離と方向)を入力とする追従制御(比例制御)
- 接続・停止・リセットのイベントによる、セッション管理(待機 / 追従中)
- 追従中のカメラ映像の録画
- 体組成・デバイス情報(体重、体脂肪率、筋肉量、脈拍、睡眠時間)のセッションごとの保存
- 信号の途切れや、セッション管理の停止での、自動停止(安全策)

## 動作の流れ

| 場面 | ロボット | 録画と情報 |
|---|---|---|
| 待機 | 停止したまま | なし |
| 接続(`connect`、デバイスが 1.0 m 以内) | 追従を始める | 録画を始め、体組成・デバイス情報を保存する |
| 停止(`stop`) | 停止して、待機に戻る | 録画を閉じて残す。情報も残す |
| リセット(`reset`) | 停止して、待機に戻る | そのセッションの録画と情報を削除する(それ以前のセッションは残す) |
| セッション管理の停止 | 約 3 秒以内に停止 | 録画を閉じて残す |

## システム構成

```
デバイス(模擬) ──相対位置──────────────► 制御 ──速度指令──► Gazebo(ロボット)
      │                                      ▲                     │
      │ 接続・停止・リセット                 │ 状態                │ カメラ画像
      ▼                                      │                     ▼
セッション管理 ──状態─────────────────────────┴──────────────► 録画
      ▲                                                            ▲
      └──── 状態(待機 / 追従中)                 体組成・デバイス情報 ┘
```

## 動作環境

- ROS 2 Humble
- Gazebo Classic(`gazebo_ros`、`gazebo_ros_diff_drive`、`gazebo_ros_camera`)
- Python 3(ノードはすべて Python)
- OpenCV(`cv2`)と numpy(録画ノードに必要)

## ディレクトリ構成

```
start_sim.sh                      Gazebo の起動とロボットの生成
ros_nodes/
  follower_controller_signal.py   追従の制御ノード(デバイスの相対位置を入力)
  session_manager.py              セッション管理(待機 / 追従中、接続・停止・リセット)
  session_recorder.py             録画と、体組成・デバイス情報の保存
  device_signal_sim.py            デバイス信号の模擬(actor の位置から相対位置を作る)
  participant_info_sim.py         体組成・デバイス情報の模擬(ダミー値)
  add_camera.py                   ロボットのモデルに、カメラと支柱を追加するパッチ
  follower_controller_new.py      追従の制御ノード(actor の位置を直接読む版。比較用)
  (そのほか)                      過去の試作(Behavior Cloning による追従の検討など)
simulation/
  models/follower_bot_new/        ロボットのモデル(model.sdf)
  models/walker/                  歩行する人物のモデル
  worlds/                         ワールド(actor_physics_test.world など)
data/                             学習用データ(試作で使用)
```

## 使い方

### 1. シミュレーションを起動する

```bash
./start_sim.sh
```

`actor_physics_test.world` で Gazebo を起動し、ロボットを (2.0, 0.0) に生成します。生成位置は、環境変数で変えられます(actor から 1.5 m 以上離してください)。

```bash
SPAWN_X=3.0 SPAWN_Y=1.5 ./start_sim.sh
```

### 2. ノードを起動する

ターミナルを分けて、それぞれ `source /opt/ros/humble/setup.bash` を実行し、`ros_nodes/` で起動します。

```bash
python3 device_signal_sim.py
python3 session_manager.py
python3 follower_controller_signal.py
python3 session_recorder.py
python3 participant_info_sim.py
```

接続の距離(既定 1.0 m)を広げて試す場合は、次のように起動します。

```bash
python3 session_manager.py --ros-args -p connect_distance:=5.0
```

### 3. イベントを送る

| やりたいこと | コマンド |
|---|---|
| 追従を始める | `ros2 topic pub -r 2 -t 3 /device/event std_msgs/msg/String "{data: connect}"` |
| 追従を止める(録画と情報は残す) | `ros2 topic pub -r 2 -t 3 /device/event std_msgs/msg/String "{data: stop}"` |
| 追従を止めて、そのセッションの録画と情報を削除する | `ros2 topic pub -r 2 -t 3 /device/event std_msgs/msg/String "{data: reset}"` |

同じイベントを 3 回送るのは、`ros2 topic pub --once` が、受信側との接続前に、メッセージを取りこぼすことがあるためです。2 回目以降は無視されます。

### 注意

- 手動の `ros2 topic pub` による速度指令や、`follower_controller_new.py` を、制御ノードと同時に動かさないでください(指令が競合します)。
- `diff_drive` プラグインは、最後の速度指令を保持します。止めたいときは、速度 0 の指令が必要です(制御ノードは、停止時に自動で送ります)。
- 位置だけ戻すときは、`reset_world` を使ってください。`reset_simulation` は、使わないでください(`/model_states` の配信が止まることがあります)。
- Gazebo を再起動したときは、ロボットを生成し直す必要があります(`start_sim.sh` が行います)。

## トピック一覧

| トピック | 型 | 内容 |
|---|---|---|
| `/device/relative_position` | `geometry_msgs/PointStamped` | デバイスの相対位置[m](ロボット座標系。x=前方、y=左) |
| `/device/event` | `std_msgs/String` | `connect` / `stop` / `reset` |
| `/session/state` | `std_msgs/String` | `IDLE` / `FOLLOWING` |
| `/session/participant_info` | `std_msgs/String`(JSON) | 体重、体脂肪率、筋肉量、脈拍、睡眠時間(と `device_id`) |
| `/follower_bot_new/cmd_vel` | `geometry_msgs/Twist` | ロボットへの速度指令 |
| `/follower_bot_new/camera/image_raw` | `sensor_msgs/Image` | カメラ画像(10 Hz、640x480) |

## 録画と情報の保存

保存先は `recordings/` です(既定 `/home/ubuntu/elderly_ai/recordings`)。このフォルダは、Git の対象外です。

- `session_YYYYmmdd_HHMMSS.mp4` : 録画
- `session_YYYYmmdd_HHMMSS.json` : そのセッションの体組成・デバイス情報

`reset` で終了したセッションの、録画と情報を、両方削除します。`stop` などの、それ以外の終了では、残します。範囲外の値や、決めた項目以外は、保存しません。健康に関わる値は、ログに出しません。フォルダは 0700、ファイルは 0600 の権限で作ります。

## 個人情報の扱い

このリポジトリのシミュレーションは、ダミーの値だけを使います。実在の人のデータは、含みません。

想定する運用(高齢者の健康維持管理)では、録画と情報を、介護施設のサーバに 1 年間保存し、リセット時に、最新のセッション分を削除します。1 年後の自動削除は、未実装です。実在の人のデータを扱う前に、本人または家族の同意、アクセス権限、通信と保存の暗号化、施設の規程との照合が必要です。

## 既知の制限と今後の課題

- 比例制御のみのため、actor が歩き続けると、距離が目標の 0.8 m より遠い、約 1.1 m で釣り合います。
- 複数の actor から、追う相手を選ぶ機能は、未実装です(接続したデバイスの ID と actor の対応づけが必要です)。
- 脈拍や睡眠時間から、健康状態を判断して通知する機能は、未実装です。
- 実機では、距離と方向が取れる方式(UWB など)のデバイスを使い、模擬ノードを、受信ノードに置き換える必要があります。
- 追う相手との間に、人や物が入った場合の、障害物の回避は、未実装です。
