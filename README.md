# db3转mp4脚本使用指南

Author: ACE_Sun_Yihang

Version: 1.0.0

Date: 2024.11.14

## 准备工作

* 在 src/rosbag_vision/launch/run_rosbag.launch.py 中第23行处修改为你要转换的 ros2bag 的 db3 文件的路径。
* 使用 Foxglove Studio 以打开本地的 db3 文件，确定图像的话题以及数据类型。（不同学校的 ros2bag 不同，例如东大的话题为 /compressed_image，数据类型为 sensor_msgs/msg/CompressedImage）
* 根据话题名称修改 src/utils/rosbag_player/rosbag_player.cpp 第62行处的话题名称。
* output.mp4 文件会自动生成在工作空间目录下。

## 运行

在工作空间下打开终端输入：

```
colcon build
. install/setup.bash
ros2 launch rosbag_vision run_rosbag.launch.py
```

然后会出现视频画面（窗体名 bag_image），等待 bag 自动播放完自动退出程序（或使用 Ctrl + C 强行中断），便可得到 output.mp4 。（播放了多少内容就保存了多少内容）

## 小问题

导出的视频并无法在 Ubuntu 中直接播放，但可以在 VSCode 中播放，推测是视频采用 avc1 编码的原因。
