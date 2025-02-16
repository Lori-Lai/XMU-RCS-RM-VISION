# hik_camera_ros2_driver

参考中南大学和北极熊战队开源对rv原驱动进行修改，具有以下新功能：

- 设置帧率、图像格式

- 录制视频
- 播放视频

## Params

- exposure_time
- gain
- pixel_format
- adcbit_depth

后两个参数为 string ，按照需要查看 hik 中的 XML 属性表，下面给出几个常见的对应示例

| 参数            | 值           | 内容          | 备注
|-----------------|--------------|---------------|------------
| pixel_format    | BayerRG8     | bayer_RG8     |
| pixel_format    | RGB8Packed   | RGB8          |
| adcbit_depth    | Bits_12      | 12bit         |
| adcbit_depth    | Bits_8       | 8bit          | 只有 pixel_format 为 BayerRG8 才可以设置

## Run

```
# camera
ros2 launch hik_camera hik_camera_launch.py

# video
ros2 launch hik_camera video_play_launch.py
```

