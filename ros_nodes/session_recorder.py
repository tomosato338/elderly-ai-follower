#!/usr/bin/env python3
"""録画ノード(ステップ4: 体組成・デバイス情報の保存に対応)。

追従中(/session/state が FOLLOWING の間)だけ、カメラ画像を録画し、
そのセッションの体組成・デバイス情報を JSON で保存する。

入力:
  /follower_bot_new/camera/image_raw (sensor_msgs/Image)
  /session/state (std_msgs/String)
      'FOLLOWING' で録画開始、それ以外で終了
  /session/participant_info (std_msgs/String、JSON)
      体重、体脂肪率、筋肉量、脈拍、睡眠時間(と、任意で device_id)
  /device/event (std_msgs/String)
      'reset' が来たら、そのセッションの録画と情報を削除する

保存するもの(output_dir の中):
  session_YYYYmmdd_HHMMSS.mp4    録画
  session_YYYYmmdd_HHMMSS.json   そのセッションの体組成・デバイス情報

録画と情報の扱い:
  - リセットで終了したセッション: 録画と情報を両方削除する
    (それ以前のセッションのものは残す)
  - それ以外の理由で終了したセッション(ノード停止、状態の途絶など):
    両方とも残す(異常終了で、意図せず消えないようにする)

プライバシー:
  - 健康に関わる値は、ログに出さない
  - 保存先のフォルダは 0700、JSON ファイルは 0600 で作る
  - output_dir は Git に上げないこと(.gitignore に recordings/ を追加)

必要なもの: python3 の cv2(OpenCV)と numpy
"""

import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


# 受け付ける値の範囲(これを外れた入力は、保存しない)
INFO_RANGES = {
    'weight_kg': (20.0, 300.0),
    'body_fat_percent': (1.0, 70.0),
    'muscle_mass_kg': (5.0, 100.0),
    'heart_rate_bpm': (20.0, 250.0),
    'sleep_hours': (0.0, 24.0),
}


def parse_participant_info(text):
    """JSON 文字列を検証する。戻り値は (情報の辞書, エラーの理由)。

    エラーの理由には、値そのものは含めない(キー名だけ)。
    """
    try:
        data = json.loads(text)
    except ValueError:
        return None, 'invalid JSON'
    if not isinstance(data, dict):
        return None, 'not a JSON object'

    info = {}
    for key, (low, high) in INFO_RANGES.items():
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None, f'missing or non-numeric: {key}'
        if not (low <= value <= high):   # NaN や inf も、ここで弾かれる
            return None, f'out of range: {key}'
        info[key] = value

    device_id = data.get('device_id')
    if isinstance(device_id, str) and 0 < len(device_id) <= 64:
        info['device_id'] = device_id
    return info, None


def session_json_path(video_path):
    return os.path.splitext(video_path)[0] + '.json'


def write_private_json(path, obj):
    """所有者だけが読み書きできる(0600)JSON を書く。"""
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


def remove_session_files(video_path):
    """セッションの録画と情報のファイルを削除し、削除したパスを返す。"""
    removed = []
    for path in (video_path, session_json_path(video_path)):
        if os.path.exists(path):
            os.remove(path)
            removed.append(path)
    return removed


def image_to_bgr(msg):
    """sensor_msgs/Image(rgb8 または bgr8)を、OpenCV 用の BGR 配列にする。"""
    h, w = msg.height, msg.width
    if msg.encoding not in ('rgb8', 'bgr8'):
        return None
    data = np.frombuffer(msg.data, dtype=np.uint8)
    frame = data.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
    if msg.encoding == 'rgb8':
        frame = frame[:, :, ::-1]
    return np.ascontiguousarray(frame)


class SessionRecorder(Node):
    def __init__(self):
        super().__init__('session_recorder')

        self.declare_parameter(
            'image_topic', '/follower_bot_new/camera/image_raw'
        )
        self.declare_parameter('session_topic', '/session/state')
        self.declare_parameter('event_topic', '/device/event')
        self.declare_parameter(
            'info_topic', '/session/participant_info'
        )
        self.declare_parameter(
            'output_dir', '/home/ubuntu/elderly_ai/recordings'
        )
        self.declare_parameter('fps', 10.0)
        self.declare_parameter('state_timeout', 3.0)   # s
        self.declare_parameter('reset_grace', 2.0)     # s
        self.declare_parameter('info_window', 10.0)    # s

        p = self.get_parameter
        self.output_dir = p('output_dir').value
        self.fps = p('fps').value
        self.state_timeout = p('state_timeout').value
        self.reset_grace = p('reset_grace').value
        self.info_window = p('info_window').value

        self.path = None            # 録画中のファイル(録画中でなければ None)
        self.writer = None
        self.frames = 0
        self.info = None            # 現在のセッションの体組成・デバイス情報
        self.pending_info = None    # (情報, 受信時刻): セッション開始前に届いた分
        self.wait_idle = False      # リセット後、IDLE を確認するまで再開しない
        self.last_closed = None     # (path, 閉じた時刻)
        self.last_state_time = time.monotonic()

        if cv2 is None:
            self.get_logger().error(
                'cv2 / numpy が見つかりません。録画できません。'
                ' (python3 -c "import cv2, numpy" で確認してください)'
            )

        self.image_sub = self.create_subscription(
            Image, p('image_topic').value, self.image_callback,
            qos_profile_sensor_data,
        )
        self.session_sub = self.create_subscription(
            String, p('session_topic').value, self.session_callback, 10
        )
        self.event_sub = self.create_subscription(
            String, p('event_topic').value, self.event_callback, 10
        )
        self.info_sub = self.create_subscription(
            String, p('info_topic').value, self.info_callback, 10
        )
        self.timer = self.create_timer(1.0, self.watchdog)

        self.get_logger().info(
            f'Recorder started: output_dir={self.output_dir}, '
            f'fps={self.fps}'
        )

    # ---- 状態 ----
    def session_callback(self, msg):
        self.last_state_time = time.monotonic()
        following = msg.data == 'FOLLOWING'

        if not following:
            self.wait_idle = False
            if self.path is not None:
                self.stop_recording(reason='session ended')
        elif self.path is None and not self.wait_idle:
            self.start_recording()

    def watchdog(self):
        # セッション管理ノードが止まったら、録画を閉じて残す
        if (
            self.path is not None
            and time.monotonic() - self.last_state_time > self.state_timeout
        ):
            self.get_logger().warn('session state lost; recording kept')
            self.stop_recording(reason='state lost')

    def event_callback(self, msg):
        if msg.data.strip().lower() == 'reset':
            self.handle_reset()

    def handle_reset(self):
        self.pending_info = None
        if self.path is not None:
            # 録画中にリセット: 閉じて、録画と情報を削除する
            path = self.path
            self.stop_recording(reason='reset', keep_record=False)
            self.delete_session(path)
            self.wait_idle = True
        elif self.last_closed is not None:
            # 状態(IDLE)がリセットより先に届いて、すでに閉じていた場合
            path, closed_at = self.last_closed
            if time.monotonic() - closed_at <= self.reset_grace:
                self.delete_session(path)
            self.last_closed = None

    # ---- 体組成・デバイス情報 ----
    def info_callback(self, msg):
        info, error = parse_participant_info(msg.data)
        if info is None:
            self.get_logger().warn(f'participant info rejected: {error}')
            return

        if self.path is not None:
            self.info = info
            self.write_info()
        else:
            # セッションの開始より先に届いた場合は、少しの間だけ保持する
            self.pending_info = (info, time.monotonic())
            self.get_logger().info(
                'participant info received (waiting for session)'
            )

    def write_info(self):
        if self.path is None or self.info is None:
            return
        record = {
            'saved_at': datetime.now().isoformat(timespec='seconds'),
            'video_file': os.path.basename(self.path),
            'participant_info': self.info,
        }
        try:
            write_private_json(session_json_path(self.path), record)
            self.get_logger().info('participant info saved')
        except OSError as e:
            self.get_logger().error(f'failed to save participant info: {e}')

    # ---- 録画 ----
    def start_recording(self):
        if cv2 is None:
            return
        os.makedirs(self.output_dir, mode=0o700, exist_ok=True)
        name = datetime.now().strftime('session_%Y%m%d_%H%M%S.mp4')
        self.path = os.path.join(self.output_dir, name)
        self.writer = None   # 最初の画像で、サイズを決めて作る
        self.frames = 0
        self.info = None
        self.last_closed = None
        self.get_logger().info(f'recording started: {self.path}')

        # 開始の直前に届いていた情報があれば、このセッションのものとする
        if self.pending_info is not None:
            info, received_at = self.pending_info
            self.pending_info = None
            if time.monotonic() - received_at <= self.info_window:
                self.info = info
                self.write_info()

    def stop_recording(self, reason, keep_record=True):
        path = self.path
        if self.writer is not None:
            self.writer.release()
            os.chmod(path, 0o600)
        frames = self.frames
        self.path = None
        self.writer = None
        self.frames = 0
        self.info = None

        if keep_record:
            self.last_closed = (path, time.monotonic())
            self.get_logger().info(
                f'recording saved ({reason}): {path} [{frames} frames]'
            )
        else:
            self.get_logger().info(f'recording stopped ({reason})')

    def delete_session(self, video_path):
        try:
            removed = remove_session_files(video_path)
            if removed:
                for path in removed:
                    self.get_logger().info(f'deleted (reset): {path}')
            else:
                self.get_logger().info(
                    f'reset: no file to delete ({video_path})'
                )
        except OSError as e:
            self.get_logger().error(f'failed to delete: {e}')

    def image_callback(self, msg):
        if self.path is None or cv2 is None:
            return

        frame = image_to_bgr(msg)
        if frame is None:
            self.get_logger().warn(
                f'unsupported encoding: {msg.encoding}',
                throttle_duration_sec=5.0,
            )
            return

        if self.writer is None:
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.path, fourcc, self.fps, (w, h))
            if not self.writer.isOpened():
                self.get_logger().error(f'cannot open writer: {self.path}')
                self.writer = None
                self.path = None
                return
            os.chmod(self.path, 0o600)

        self.writer.write(frame)
        self.frames += 1

    def shutdown_recording(self):
        """ノード終了時: 録画中なら、閉じて残す。"""
        if self.path is not None:
            self.stop_recording(reason='node shutdown')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = SessionRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown_recording()
        except Exception:
            pass

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
