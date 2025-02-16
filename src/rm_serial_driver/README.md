# rm_serial_driver
RoboMaster 视觉系统与电控系统的串口通讯模块

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

该项目为 [rm_vision](https://github.com/chenjunnn/rm_vision) 的子模块

## Overview

本模块基于 [transport_drivers](https://github.com/ros-drivers/transport_drivers) 实现了 [rm_auto_aim](https://github.com/chenjunnn/rm_auto_aim) 项目与电控部分通讯的功能，也可作为使用 ros2 作为开发框架的参赛队的串口通讯模块参考

若有帮助请Star这个项目，感谢~

## 使用指南

安装依赖 `sudo apt install ros-humble-serial-driver`

更改 [serial_driver.yaml](config/serial_driver.yaml) 中的参数以匹配与电控通讯的串口

启动串口模块 `ros2 launch rm_serial_driver serial_driver.launch.py`

## 发送和接收

详情请参考 [packet.hpp](include/rm_serial_driver/packet.hpp)

从电控端需要接受

- 机器人的自身颜色 `robot_color` 以判断对应的识别目标颜色
- 云台姿态 `pitch` 和 `yaw`, 单位和方向请参考 https://www.ros.org/reps/rep-0103.html
- 当前云台瞄准的位置 `aim_x, aim_y, aim_z`，用于发布可视化 Marker

视觉端发送 armor_solver 的输出，即云台控制信息

## 电控端的处理

TBD
