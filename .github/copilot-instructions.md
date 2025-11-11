## 快速上手 — 给 AI 编码代理的要点

面向在此仓库中补全、修复或实现功能的 AI 代理，着重收集已被代码验证的事实与工作流。

### 架构速览
- ROS2 Humble + ament_cmake，目标环境 Ubuntu 22.04。
- 典型自瞄链路：`hik_camera` → `armor_detector` → `armor_tracker` → `armor_solver` → `rm_serial_driver`，由 `rm_vision_bringup/vision_bringup.launch.py` 编排。
- 自定义消息全部在 `src/rm_auto_aim/auto_aim_interfaces/msg`，核心为 `Armors`, `Target`, `GimbalCmd`。

### 数据流与关键节点
- `hik_camera` 节点向 `/image_raw` 发布图像，参数集中在 `node_params.yaml` 下 `/hik_camera` 节点；帧率与曝光控制高频调优。
- `armor_detector` (`armor_detector/src/detector_node.cpp`) 订阅 `/image_raw`；`Detector` 通过 `model/mlp.onnx` 分类装甲号，`PnPSolver`+`BaOptimizer` 求位姿，输出 `/detector/armors` 与调试话题 `/detector/result_img` 等。`use_ba` 参数控制 BA 优化，只在滚转角小于 15° 时触发。
- `armor_tracker` (`armor_tracker/src/tracker_node.cpp`) 使用扩展卡尔曼滤波跟踪整车状态（9 维状态: 位置/速度/姿态/半径），对 `tracker.*` 和 `ekf.*` 参数敏感；输出 `/tracker/target`，并在 `/tracker/reset` 服务重置状态。
- `armor_solver` (`armor_solver/src/armor_solver.cpp`) 按 `CompensatorFactory` 类型执行弹道补偿（`ideal` 或 `resistance`），结合手动补偿表 `solver.angle_offset`。最终发布 `/solver/cmd_gimbal`。
- `rm_serial_driver` (`rm_serial_driver/src/rm_serial_driver.cpp`) 与下位机串口通讯：发布 `gimbal_link` TF，按 MCU 请求更新 `armor_detector/detect_color` 参数，并把 `/cmd_vel` / `/solver/cmd_gimbal` 写回串口；CRC 失效会自动重连。

### 构建与运行
- 依赖安装：`rosdep install --from-paths src --ignore-src -r -y`，额外手动编译 abseil, ceres 2.1.0, Sophus 1.24.6, g2o（参见 `/README.md`）。
- 发布构建：`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`；单包调试：`colcon build --packages-select armor_solver --symlink-install`。
- 运行流程：`source install/setup.bash` 后，`ros2 launch rm_vision_bringup vision_bringup.launch.py`。需要串口权限：`sudo usermod -aG dialout <username>`。
- 单包测试示例：`colcon test --packages-select armor_solver`；跟踪链路可用 `ros2 topic echo /tracker/target` 验证输出。

### 代码与参数约定
- 默认改动通过 `src/rm_vision_bringup/config/node_params.yaml` 调整运行参数；修改源码前确认是否已有参数入口。
- 各节点按组件形式编写（`rclcpp_components` 宏注册）；新增节点需在对应 `CMakeLists.txt` 注册组件及插件描述。
- 坐标系约定：`odom` → `gimbal_link` → 相机，`armor_detector` 中 `R_gimbal_camera_` 提供从云台到相机的固定旋转；`rm_serial_driver` 广播 `gimbal_link` TF。
- `auto_aim_interfaces` 改动会影响所有依赖包，修改消息后需全量 `colcon build` 并更新 `CMakeLists.txt` 的依赖。

### 调试抓手
- 图像/检测调试：打开 `/armor_detector` 的 `debug` 参数以发布二值图/分类图；`rqt_image_view` 可查看 `/detector/result_img`。
- 跟踪状态：`/tracker/info` Marker & Msg 揭示 EKF 测量误差；异常跳板优先检查 `ekf.sigma2_q_*` 与 `tracker.*` 阈值。
- 解算与串口：`armor_solver` 在 `RCLCPP_DEBUG` 打印俯仰零点调试信息；串口异常时查看 `rm_serial_driver` 自动重连日志并确认 `timestamp_offset`。

### 其他提示
- `startup.bash` 为上电自启动脚本样例；若需要在比赛机上自启，确保路径与权限正确。
- 构建目录 `build/` 与 `install/` 可能残留旧配置；切换分支或依赖后可执行 `colcon build --packages-select <pkg> --cmake-args -DCMAKE_BUILD_TYPE=Release --no-warn-unused-cli` 触发重新配置。
- 引入新依赖时遵循 ament 导入方式：在 `package.xml` 添加 `<depend>`，在 `CMakeLists.txt` 的 `find_package` 与 `ament_target_dependencies` 中声明。
