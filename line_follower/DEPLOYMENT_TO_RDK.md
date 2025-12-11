# RDK X5 部署与运行指南

下面的步骤基于本仓库新增的 `line_follower_control` 包，结合已有的感知节点实现线速度/角速度推理后，再将其转换为左右轮转速并通过 USB-RS485 下发到电机驱动器。

## 1. 推送代码到 RDK

1. 将 RDK 通过网线或 Wi-Fi 接入同一局域网，确认其 IP 地址（例如 192.168.1.88）。
2. 在开发机上使用 SSH 推送代码：

```bash
# 如有需要先生成密钥
ssh-keygen -t ed25519
ssh-copy-id root@192.168.1.88

# 推送仓库
scp -r ~/ROS2_RDK root@192.168.1.88:~/ROS2_RDK
```

如果 RDK 已经有工作空间，也可以仅推送 `line_follower` 目录：

```bash
scp -r ~/ROS2_RDK/line_follower root@192.168.1.88:~/line_follower_ws/src/
```

## 2. 准备 ROS 环境

在 RDK 上连接终端（SSH 或串口）：

```bash
# 进入源码工作空间
cd ~/line_follower_ws

# 选择对应的 TROS 版本
source /opt/tros/humble/local_setup.bash   # Humble
# 或
source /opt/tros/local_setup.bash          # Foxy
```

确保依赖已安装（pyserial、rclpy 等在系统镜像中通常自带，如缺失可 apt 安装）：

```bash
sudo apt update
sudo apt install -y python3-serial
```

## 3. 编译工作空间

```bash
cd ~/line_follower_ws
colcon build --symlink-install
source install/setup.bash
```

## 4. 运行感知与控制

1. 启动相机与感知节点（参考仓库 README 已有步骤）。
2. 在新的终端运行差速控制节点，并加载默认参数：

```bash
source ~/line_follower_ws/install/setup.bash
ros2 run line_follower_control differential_drive_controller \
  --ros-args --params-file install/line_follower_control/share/line_follower_control/config/control_params.yaml
```

### 可调参数

- `wheel_diameter`：车轮直径（米），默认 0.14。
- `wheel_base`：左右轮距（米），默认 0.5。
- `max_linear_speed`：限幅线速度（m/s），默认 1.11（约 4 km/h）。
- `rated_motor_rpm`：电机额定转速，默认 2750。
- `kp/ki/kd`：PID 参数，控制左右轮 rpm 跟踪。
- `port`、`baudrate`：USB-RS485 串口号和波特率。
- `camera_fov_deg`、`image_width`：用于根据中心点像素计算偏航角。

感知节点仍发布 `/cmd_vel`；如果提供 `line_center`（`geometry_msgs/msg/Point`，x 为像素列，y 可留 0），控制节点会自动将像素偏移转换为转向角并生成左右轮转速，通过 RS-485 帧发送到驱动器。

## 5. 运行中的检查

- 使用 `ros2 topic echo /cmd_vel` 确认感知输出。
- 使用 `ros2 topic echo /line_center` 查看中心点（如有）。
- 通过 USB-RS485 接线连接驱动器，驱动器协议默认帧格式：`0xAA 0x55 | left_rpm | right_rpm | checksum`，如需自定义可在 `differential_drive_controller.py` 中修改。

## 6. 常见问题

- **无法打开串口**：检查 `port` 参数是否与 `/dev/ttyUSB*` 一致，确认 `dialout` 权限。
- **车体偏航大**：降低 `max_linear_speed` 或调低 `kp`，增大 `kd` 以抑制超调。
- **方向反了**：交换驱动器接线或在 RS-485 帧发送前交换左右轮 rpm。
