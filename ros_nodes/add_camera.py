#!/usr/bin/env python3
"""model.sdf に、支柱つきのカメラを追加する(ステップ3)。

使い方:
  python3 add_camera.py [model.sdf のパス]

- すでに camera_link があれば、何も変更しない
- 変更前のファイルを、model.sdf.before_camera.bak として残す
- 追加するもの: 支柱とカメラの link(camera_link)、base_link への固定 joint、
  カメラ(10 Hz、640x480、水平視野 80 度)と gazebo_ros_camera プラグイン
- 画像のトピック: /follower_bot_new/camera/image_raw
"""

import sys

DEFAULT_PATH = (
    '/home/ubuntu/elderly_ai/simulation/models/'
    'follower_bot_new/model.sdf'
)

CAMERA_XML = """
    <!-- ステップ3: 支柱つきカメラ(前方 +x を向く) -->
    <link name="camera_link">
      <pose>0.10 0 0.80 0 0 0</pose>
      <inertial>
        <mass>0.05</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <iyy>0.0001</iyy>
          <izz>0.0001</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="camera_mast_visual">
        <pose>0 0 -0.30 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.01</radius>
            <length>0.60</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.3 0.3 0.3 1</ambient>
          <diffuse>0.3 0.3 0.3 1</diffuse>
        </material>
      </visual>
      <visual name="camera_body_visual">
        <geometry>
          <box>
            <size>0.04 0.06 0.04</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.1 1</ambient>
          <diffuse>0.1 0.1 0.1 1</diffuse>
        </material>
      </visual>
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>10</update_rate>
        <camera>
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>20</far>
          </clip>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <ros>
            <namespace>/follower_bot_new</namespace>
          </ros>
          <camera_name>camera</camera_name>
          <frame_name>camera_link</frame_name>
        </plugin>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <pose>0 0 0 0 0 0</pose>
      <parent>base_link</parent>
      <child>camera_link</child>
    </joint>
"""


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PATH
    text = open(path, encoding='utf-8').read()

    if 'camera_link' in text:
        print('すでにカメラが追加されています。変更しません。')
        return 0

    index = text.rfind('</model>')
    if index < 0:
        print('</model> が見つかりません。SDF の形式を確認してください。')
        return 1

    backup = path + '.before_camera.bak'
    with open(backup, 'w', encoding='utf-8') as f:
        f.write(text)

    with open(path, 'w', encoding='utf-8') as f:
        f.write(text[:index].rstrip() + '\n' + CAMERA_XML + '  ' + text[index:])

    print(f'カメラを追加しました: {path}')
    print(f'元のファイル: {backup}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
