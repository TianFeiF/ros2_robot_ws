# rm_force_publisher

RM75 七自由度机械臂六维力传感器重力补偿与发布节点。

订阅机械臂原始力数据和末端位姿，完成夹爪重力补偿后将力/力矩变换到基座坐标系，以标准 ROS2 话题持续发布。

---

## 环境依赖

- ROS2 Humble
- 机械臂驱动包（`rm_driver`、`rm_ros_interfaces`）已安装并可正常运行
- Python 依赖：

```bash
pip install numpy scipy pyyaml
```

> 若出现 `numpy.dtype size changed` 错误，说明系统 scipy 与 pip numpy 版本不兼容，执行以下命令修复：
> ```bash
> pip install --upgrade scipy
> ```

---

## 包结构

```
rm_force_publisher/
├── config/
│   └── force_calibration.yaml   # 夹爪重力标定参数
├── launch/
│   └── force_publisher.launch.py
├── rm_force_publisher/
│   └── force_publisher_node.py  # 发布节点源码
├── package.xml
├── setup.py
└── README.md
```

---

## 安装

将本包复制到工作空间的 `src/` 目录下，然后编译：

```bash
cd ~/ros2_robot_ws
colcon build --packages-select rm_force_publisher
source install/setup.bash
```

---

## 运行

机械臂驱动必须已启动，本节点才能收到数据。

**终端 1：启动机械臂驱动**
```bash
cd ~/ros2_robot_ws
source install/setup.bash
ros2 launch rm_bringup rm_75_bringup.launch.py
```

**终端 2：启动力数据发布节点**
```bash
cd ~/ros2_robot_ws
source install/setup.bash
ros2 launch rm_force_publisher force_publisher.launch.py
```

启动成功后终端输出：
```
[force_publisher]: force_publisher 已启动，标定文件: .../force_calibration.yaml，等待数据...
```

---

## 发布的话题

| 话题 | 消息类型 | 频率 | 说明 |
|------|----------|------|------|
| `/compensated_wrench` | `geometry_msgs/WrenchStamped` | 50 Hz | 重力补偿后的力/力矩，基座坐标系 |

### 消息字段

```
geometry_msgs/WrenchStamped
├── header
│   ├── stamp       # ROS 时间戳
│   └── frame_id    # 固定为 "base_link"
└── wrench
    ├── force
    │   ├── x       # 单位 N
    │   ├── y
    │   └── z
    └── torque
        ├── x       # 单位 N.m
        ├── y
        └── z
```

**坐标系方向（基座坐标系）：**
- X 轴：机械臂正前方
- Y 轴：机械臂左侧
- Z 轴：竖直向上
- 力矩方向符合右手定则

**正常状态参考值（无外力接触时）：**
- `force` 各分量 < ±0.5 N
- `torque` 各分量 < ±0.1 N.m

---

## 接收数据

### 终端直接查看

```bash
ros2 topic echo /compensated_wrench
```

### Python 节点

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class ForceReceiver(Node):
    def __init__(self):
        super().__init__('force_receiver')
        self.create_subscription(
            WrenchStamped, '/compensated_wrench', self.callback, 10)

    def callback(self, msg):
        fx = msg.wrench.force.x
        fy = msg.wrench.force.y
        fz = msg.wrench.force.z
        mx = msg.wrench.torque.x
        my = msg.wrench.torque.y
        mz = msg.wrench.torque.z
        self.get_logger().info(
            f"F=[{fx:+.3f}, {fy:+.3f}, {fz:+.3f}]N  "
            f"M=[{mx:+.3f}, {my:+.3f}, {mz:+.3f}]N.m")

def main():
    rclpy.init()
    rclpy.spin(ForceReceiver())
    rclpy.shutdown()
```

### C++ 节点

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

class ForceReceiver : public rclcpp::Node
{
public:
    ForceReceiver() : Node("force_receiver")
    {
        sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/compensated_wrench", 10,
            [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                RCLCPP_INFO(get_logger(),
                    "F=[%.3f, %.3f, %.3f]N  M=[%.3f, %.3f, %.3f]N.m",
                    msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z,
                    msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
            });
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceReceiver>());
    rclcpp::shutdown();
}
```

### 多机接收

两台电脑需在同一局域网，`ROS_DOMAIN_ID` 保持一致（默认为 0）：

```bash
export ROS_DOMAIN_ID=0
ros2 topic echo /compensated_wrench
```

---

## 标定参数说明

标定参数存放于 `config/force_calibration.yaml`，由标定程序自动生成，**请勿手动修改**。

| 字段 | 说明 | 单位 |
|------|------|------|
| `mass` | 夹爪质量 | kg |
| `com` | 夹爪重心位置（传感器坐标系） | m |
| `f_bias` | 力传感器零偏 | N |
| `m_bias` | 力矩传感器零偏 | N.m |

若更换夹爪或重新标定，将新的 `force_calibration.yaml` 替换 `config/` 目录下的文件，重新编译后生效：

```bash
colcon build --packages-select rm_force_publisher
```

---

## 常见问题

**Q：启动后一直显示"等待数据"？**
A：确认机械臂驱动已启动，检查话题是否存在：
```bash
ros2 topic list | grep rm_driver
```

**Q：`numpy.dtype size changed` 报错？**
A：scipy 与 numpy 版本不兼容，执行：
```bash
pip install --upgrade scipy
```

**Q：收不到 `/compensated_wrench` 话题？**
A：检查 `ROS_DOMAIN_ID` 是否一致，以及两台电脑是否在同一局域网。
