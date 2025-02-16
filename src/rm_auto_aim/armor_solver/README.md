# armor_solver

弹道解算、火控

### 发布话题 

* `solver/measurement` (`auto_aim_interfaces/msg/Measurement`) - EKF的输入观测量
* `solver/cmd_gimbal` (`auto_aim_interfaces/msg/GimbalCmd`) - 云台控制指令

### 订阅话题

*  `tracker/target` (`auto_aim_interfaces/msg/Target`) - 跟踪目标

  
### 参数 

* `solver.debug` (`bool`, default: false) - 是否发布调试信息
* `solver.prediction_delay` (`double`, default: 0.0) - 预测延迟时间（s），会影响选版
* `solver.controller_delay` (`double`, default: 0.0) - 控制延迟时间（s），不会影响选版
* `solver.max_tracking_v_yaw` (`double`, default: 60.0) - 转速大于这个值时，瞄准中心
* `solver.side_angle` (`double`, default: 15.0) - 跳转到下一装甲板的角度阈值
* `solver.bullet_speed` (`double`, default: 25.0) - 子弹速度
* `solver.gravity` (`double`, default: 9.8) - 重力加速度
* `solver.compensator_type` (`string`, default: "ideal") - 补偿器类型
* `solver.resistance` (`double`, default: 0.001) - 空气阻力