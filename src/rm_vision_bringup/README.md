# rm_vision

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

## Overview

本项目不存在任何交流群、交流板块、论坛等。任何自称“rm_vision交流”等的群组、论坛等均为假冒，请注意辨别。

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![State-of-the-art Shitcode](https://img.shields.io/static/v1?label=State-of-the-art&message=Shitcode&color=7B5804)](https://github.com/trekhleb/state-of-the-art-shitcode)

如您所见，本项目带有“Shitcode”标志，这说明本项目极其不适合投入实际生产，仅供交流学习之用途，如果擅自使用，后果自负。

## 包含项目

装甲板自动瞄准算法模块 https://gitlab.com/rm_vision/rm_auto_aim

MindVision 相机模块 https://gitlab.com/rm_vision/ros2_mindvision_camera

HikVision 相机模块 https://gitlab.com/rm_vision/ros2_hik_camera

机器人云台描述文件 https://gitlab.com/rm_vision/rm_gimbal_description

串口通讯模块 https://gitlab.com/rm_vision/rm_serial_driver

视觉算法仿真器 https://gitlab.com/rm_vision/rm_vision_simulator

## 通过 Docker 部署

拉取镜像

```
docker pull hezhexi2002/rm_vision:backup
```

构建开发容器

```
docker run -it --name rv_devel \
--privileged --network host \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ws:/ros_ws \
hezhexi2002/rm_vision:backup \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

构建运行容器

```
docker run -it --name rv_runtime \
--privileged --network host --restart always \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ws:/ros_ws \
hezhexi2002/rm_vision:backup \
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

TBD

## 源码编译

TBD
