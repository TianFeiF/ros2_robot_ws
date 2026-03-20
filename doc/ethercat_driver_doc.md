# EtherCAT Device Control Driver Documentation

This document details the implementation principles, code structure, and usage guide for the `ethercat_device_control` package.

## 1. Overview

`ethercat_device_control` is a `ros2_control` hardware interface plugin (SystemInterface) for controlling 5 EYOU servo motors via EtherCAT bus. It utilizes the IgH EtherCAT Master (`ethercat-master`) library for underlying communication.

### Key Features
- **Hybrid Control Modes**: Supports 5 real EtherCAT axes + optional loopback/dummy axes.
- **CiA 402 State Machine**: Built-in management of the CiA 402 state machine (Shutdown -> Switch On -> Operation Enabled).
- **Safety Protection**: Prevents "jump" or "runaway" behavior by syncing command positions with actual positions before enabling.
- **Blocking Zero Calibration**: Performs a safe, blocking calibration sequence for the vertical axis (Slave 0) before handing control to ROS 2.
- **Software Offset**: Implements zero-offset logic in software, avoiding hardware EEPROM/Register dependencies.

## 2. Implementation Details

Core logic is located in `src/ethercat_hardware_interface.cpp` and `include/ethercat_device_control/ethercat_hardware_interface.hpp`.

### 2.1 Joint Mapping

The code defines `JOINT_TO_SLAVE_MAP` to map URDF joint names to EtherCAT Slave Indices (0-4).

```cpp
const std::map<std::string, int> JOINT_TO_SLAVE_MAP = {
    {"straight_joint", 0},          // Vertical Axis -> Slave 0
    {"front_wheel_joint", 1},       // Front Wheel -> Slave 1
    {"rear_wheel_joint", 2},        // Rear Wheel -> Slave 2
    {"rear_clamp_wheel_joint", 3},  // Rear Clamp -> Slave 3
    {"front_clamp_wheel_joint", 4}  // Front Clamp -> Slave 4
};
```

### 2.2 PDO Configuration

The PDO configuration matches the drive's requirements. Note that `0x607C` (Home Offset) is defined but **no longer used** as we switched to Software Offsets.

*   **RxPDO (Master -> Slave)**:
    *   `0x6040`: Controlword
    *   `0x607A`: Target Position
    *   `0x60FF`: Target Velocity
    *   `0x6071`: Target Torque
    *   `0x6060`: Modes of Operation
*   **TxPDO (Slave -> Master)**:
    *   `0x6041`: Statusword
    *   `0x6064`: Actual Position
    *   `0x606C`: Actual Velocity
    *   `0x6077`: Actual Torque

### 2.3 Zero Calibration (Blocking Mode)

For `straight_joint` (Slave 0), a blocking calibration sequence is executed during the `on_activate` phase. This ensures the hardware is homed and safe before ROS 2 controllers start.

**Logic Flow (`handle_zero_calibration`):**

1.  **Initialize**:
    - Set internal Software Offset to 0.
2.  **Move Away (Ramping Up)**:
    - Switch to **CSV (Cyclic Synchronous Velocity)** mode.
    - Ramp up velocity to `50,000 pulses/s`.
    - Run for 2.0 seconds to move away from the hard stop.
3.  **Move Close (Ramping Down)**:
    - Ramp velocity down to `-50,000 pulses/s`.
    - Monitor `Actual Torque`.
    - **Trigger Condition**: Absolute torque > `250` (Hard Stop detected).
4.  **Calculate Offset**:
    - Record the current hardware position as the "Hard Stop Position".
    - Save this value as `hw_home_offset_`.
5.  **Move to Zero**:
    - The calibration finishes.
    - The driver switches back to **CSP (Cyclic Synchronous Position)** mode.
    - ROS 2 Control takes over.

**Software Offset Mechanism:**
- **Read**: `ROS Position = (Hardware Position - Offset) / Scale`
- **Write**: `Hardware Target = (ROS Command * Scale) + Offset`

This ensures that `ROS Position = 0` corresponds exactly to the calibrated hard stop (or defined zero), regardless of the motor's absolute encoder value.

### 2.4 Startup Sequence

In `on_activate()`:
1.  Activate EtherCAT Master.
2.  Wait for all slaves to reach `OP` state.
3.  **Enter Blocking Calibration Loop**:
    - The driver halts the `on_activate` return until calibration completes.
    - During this loop, it handles EtherCAT communication and the calibration state machine.
4.  Once calibration is `DONE`, `on_activate` returns `SUCCESS`, allowing `ros2_control` to start controllers.

## 3. How to Control

System controllers:
1.  **plan_group_controller** (straight_joint + arm joints)
2.  **base_controller** (wheels)

### 3.1 Launching

```bash
ros2 launch ethercat_device_control ethercat_bringup.launch.py
```

Wait for the log: `[EthercatHardwareInterface]: Blocking Calibration Done!`

### 3.2 CLI Control Examples

**Control Wheels (base_controller):**
```bash
ros2 topic pub --once /base_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['front_wheel_joint', 'rear_wheel_joint', 'front_clamp_wheel_joint', 'rear_clamp_wheel_joint'],
  points: [
    { positions: [3.14, 3.14, 0.0, 0.0], time_from_start: { sec: 2, nanosec: 0 } }
  ]
}"
```

**Control Vertical Axis (plan_group_controller):**
```bash
ros2 topic pub --once /plan_group_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['straight_joint', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'],
  points: [
    { positions: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: { sec: 2, nanosec: 0 } }
  ]
}"
```

## 4. Troubleshooting

*   **Calibration Timeout**: If "Calibration Timeout" appears in logs, check if the motor is mechanically blocked or if torque limits are too high.
*   **Controller Load Failure**: Ensure calibration completes successfully. If calibration hangs, the controllers won't start.
*   **EtherCAT Error**: Use `dmesg` or `ethercat slaves` to check link status.
