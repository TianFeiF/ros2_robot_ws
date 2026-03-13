# EtherCAT Device Control 驱动说明文档

本文档详细说明了 `ethercat_device_control` 功能包的实现原理、代码结构以及如何控制电机运动。

## 1. 功能包概述

`ethercat_device_control` 是一个基于 `ros2_control` 的硬件接口插件 (SystemInterface)，用于通过 EtherCAT 总线控制 5 个 EYOU 伺服电机。它使用了 IgH EtherCAT Master (`ethercat-master`) 库进行底层通信。

### 关键特性
- **混合控制模式**：支持 5 个真实的 EtherCAT 轴 + 若干个模拟轴（Loopback）。
- **CiA 402 状态机**：内置了完整的 CiA 402 状态机管理（自动从 Shutdown -> Switch On -> Operation Enabled）。
- **安全保护**：防止在上电瞬间因目标位置为 0 而导致的飞车现象。
- **动态配置**：通过 URDF 参数 `use_dummy_mode` 可切换至纯模拟模式。

## 2. 代码实现详解

核心代码位于 `src/ethercat_hardware_interface.cpp` 和 `include/ethercat_device_control/ethercat_hardware_interface.hpp`。

### 2.1 关节映射 (Joint Mapping)

代码中定义了 `JOINT_TO_SLAVE_MAP`，将 URDF 中的关节名称映射到 EtherCAT 总线上的从站索引 (Slave Index 0-4)。

```cpp
const std::map<std::string, int> JOINT_TO_SLAVE_MAP = {
    {"straight_joint", 0},          // 升降关节 -> Slave 0
    {"front_wheel_joint", 1},       // 前轮 -> Slave 1
    {"rear_wheel_joint", 2},        // 后轮 -> Slave 2
    {"rear_clamp_wheel_joint", 3},  // 后夹轮 -> Slave 3
    {"front_clamp_wheel_joint", 4}  // 前夹轮 -> Slave 4
};
```

### 2.2 PDO 配置 (PDO Configuration)

PDO 配置严格匹配了电机驱动器的要求（参考了 `ethercat cstruct` 和厂家示例 `main.cpp`）。

*   **RxPDO (Master -> Slave)**:
    *   `0x6040`: Controlword (控制字)
    *   `0x607A`: Target Position (目标位置)
    *   `0x60FF`: Target Velocity (目标速度)
    *   `0x6071`: Target Torque (目标力矩)
    *   `0x6060`: Modes of Operation (操作模式)
    *   `0x607C`: Home Offset (原点偏移量)
*   **TxPDO (Slave -> Master)**:
    *   `0x6041`: Statusword (状态字)
    *   `0x6064`: Actual Position (实际位置)
    *   `0x606C`: Actual Velocity (实际速度)
    *   `0x6077`: Actual Torque (实际力矩)

### 2.3 标准状态机逻辑 (Standard State Machine)

在 `read()` 函数中，驱动程序会读取每个电机的 `Statusword`，并根据 CiA 402 标准自动发送对应的 `Controlword` 以进入 Operation Enabled 状态：

1.  **Fault (Bit 3)** -> 发送 `0x80` (Fault Reset)
2.  **Switch On Disabled** -> 发送 `0x06` (Shutdown)
3.  **Ready to Switch On** -> 发送 `0x07` (Switch On)
4.  **Switched On** -> 发送 `0x0F` (Enable Operation)
5.  **Operation Enabled** -> 保持 `0x0F`，并开始更新位置命令。

### 2.4 零点校准逻辑 (Zero Calibration - Slave 0 Only)

针对 `straight_joint` (Slave 0)，在进入 `Operation Enabled` 后会执行特殊的零点校准流程：

1.  **初始化 (INIT)**:
    - 通过 PDO 写入 `0` 到 `0x607C` (Home Offset)。
    - 进入力矩模式 (Mode 10)。

2.  **寻找硬限位 (LOOP)**:
    - 以 `200` 的目标力矩运行 2 秒（远离零点）。
    - 以 `-300` 的目标力矩反向运行，直到检测到实际力矩小于 `-302`（接触硬限位）。
    - 记录当前位置。
    - 重复 3 次。

3.  **计算平均值 (CALC)**:
    - 计算 3 次记录位置的平均值。
    - 通过 PDO 写入 `offset` 到 `0x607C` (Home Offset)。
    - 进入 **稳定状态 (STABILIZE)**。

4.  **稳定状态 (STABILIZE)**:
    - 强制失能电机 (Controlword `0x06`)，切断动力输出。
    - 等待 1 秒，让电机完全释放并稳定。
    - 进入 **同步状态 (MOVE_TO_ZERO)**。

5.  **同步状态 (MOVE_TO_ZERO)**:
    - 读取当前实际位置（此时已包含 Offset）。
    - 强制将目标位置指令设为 `0`，并写入 PDO。
    - 切换回 **CSP 模式 (8)**。
    - 标记校准完成 (DONE)。

6.  **正常运行 (DONE)**:
    - 状态机自动重新使能电机 (Shutdown -> Switch On -> Enable Op)。
    - 电机平滑运动到零点。

### 2.5 启动流程 (Startup)

在 `on_activate()` 阶段，驱动程序会执行以下步骤：
1.  激活 EtherCAT 主站。
2.  进入一个循环，等待所有 5 个从站的状态变为 `OP` (Operational)。
3.  如果 5 秒内未全部进入 OP，虽然会继续运行，但在日志中会有提示。

## 3. 如何控制电机

本系统配置了三个控制器组：
1.  **plan_group_controller** (包含 `straight_joint` 和机械臂关节)
2.  **hand_group_controller** (机械手)
3.  **base_controller** (底盘轮子)

EtherCAT 电机分布在 `plan_group_controller` (straight_joint) 和 `base_controller` (四个轮子) 中。

### 3.1 启动系统

```bash
# 终端 1
source install/setup.bash
ros2 launch ethercat_device_control ethercat_bringup.launch.py
```

等待终端输出 `所有从站已进入 OP 模式!`，并且 MoveIt/RViz 界面加载完成。

### 3.2 验证控制器状态

打开新终端，检查控制器是否激活：

```bash
ros2 control list_controllers
```

应该看到 `plan_group_controller`, `base_controller`, `joint_state_broadcaster` 处于 `active` 状态。

### 3.3 方法一：使用命令行发送指令 (CLI)

您可以使用 `ros2 topic pub` 直接向控制器发送轨迹命令。

**示例 1：控制底盘轮子 (base_controller)**
让前轮 (`front_wheel_joint`) 转动到 3.14 弧度 (约半圈)：

```bash
ros2 topic pub --once /base_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['front_wheel_joint', 'rear_wheel_joint', 'front_clamp_wheel_joint', 'rear_clamp_wheel_joint'],
  points: [
    { positions: [3.14, 3.14, 0.0, 0.0], time_from_start: { sec: 2, nanosec: 0 } }
  ]
}"
```

**示例 2：控制升降关节 (plan_group_controller)**
让升降轴 (`straight_joint`) 移动到 0.1 米：

```bash
# 注意：plan_group_controller 包含多个关节，命令中必须包含该组所有关节的名称，或者使用 partial update (取决于控制器配置)
# 为简单起见，这里假设只动 straight_joint，其他保持 0
ros2 topic pub --once /plan_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['straight_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'],
  points: [
    { positions: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 2, nanosec: 0 } }
  ]
}"
```

### 3.4 方法二：使用 MoveIt (RViz)

1.  在 RViz 界面中，找到 "MotionPlanning" 面板。
2.  在 "Planning Group" 下拉菜单中选择 `plan_group`。
3.  拖动机器人末端或使用 "Joints" 标签页拖动 `straight_joint` 的滑块。
4.  点击 "Plan & Execute"。
5.  MoveIt 将规划轨迹并通过 `plan_group_controller` 下发给硬件。

注意：`base_controller` (轮子) 可能没有配置在 MoveIt 的 `plan_group` 中，如果需要通过 MoveIt 控制轮子，需要在 SRDF 中配置相应的组，或者直接使用 CLI 控制。

## 4. 故障排查

*   **电机不转，状态一直是 0x00**：检查 EtherCAT 网线连接，确认 `ethercat slaves` 能看到所有从站。
*   **Invalid sync manager configuration**：通常是 PDO 配置与从站 EEPROM 不匹配，请重新运行 `sudo ethercat cstruct -p 0` 确认配置。
*   **飞车 (一上电就猛转)**：检查 `read()` 函数中的 "Ready to Switch On" 逻辑，确保正确回读了 `Actual Position` 并赋值给 `Target Position`。
