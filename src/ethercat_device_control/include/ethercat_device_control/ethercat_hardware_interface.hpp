#ifndef ETHERCAT_DEVICE_CONTROL__ETHERCAT_HARDWARE_INTERFACE_HPP_
#define ETHERCAT_DEVICE_CONTROL__ETHERCAT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

// Include EtherCAT master library
#include <ecrt.h>

namespace ethercat_device_control
{

// 辅助结构体：存储每个从站的PDO偏移量
struct PdoOffset {
    std::vector<unsigned int> control_word;
    std::vector<unsigned int> target_position;
    std::vector<unsigned int> target_velocity;
    std::vector<unsigned int> target_torque;
    std::vector<unsigned int> mode_of_operation;
    std::vector<unsigned int> home_offset;
    std::vector<unsigned int> dummy_byte_1;

    std::vector<unsigned int> status_word;
    std::vector<unsigned int> actual_position;
    std::vector<unsigned int> actual_velocity;
    std::vector<unsigned int> actual_torque;
    std::vector<unsigned int> mode_of_operation_display;
    std::vector<unsigned int> error_code;
    std::vector<unsigned int> dummy_byte_2;
};

// 零点校准状态机枚举
enum class CalibState {
  IDLE,               // 闲置/正常运行
  INIT,               // 初始化校准
  WRITE_ZERO_OFFSET,  // 写 0 到 0x607C
  WAIT_WRITE_ZERO,    // 等待写 0 完成
  READ_ZERO,          // Trigger read 0x607C
  WAIT_READ_ZERO,     // Wait read 0x607C
  LOOP_START,         // 循环开始
  MOVE_AWAY,          // 远离零点 (200力矩)
  MOVE_CLOSE,         // 接近零点 (-260力矩)
  RECORD_POS,         // 记录位置
  CALC_OFFSET,        // 计算偏移量
  WRITE_OFFSET,       // 写入偏移量到 0x607C
  WAIT_WRITE_OFFSET,  // 等待写入完成
  READ_OFFSET,        // Trigger read 0x607C
  WAIT_READ_OFFSET,   // Wait read 0x607C
  STABILIZE,          // 等待系统稳定 (1s)
  MOVE_TO_ZERO,       // 运动到零点
  DONE                // 完成
};

class EthercatHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 状态存储 (反馈)
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_states_effort_;

  // 命令存储 (控制)
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;
  
  // Home Offset persistent storage
  std::vector<int32_t> hw_home_offset_;

  // EtherCAT specific variables
  ec_master_t *master_ = nullptr;
  ec_domain_t *domain_ = nullptr;
  uint8_t *domain_pd_ = nullptr;
  std::vector<ec_slave_config_t*> slave_config_;
  
  PdoOffset pdo_offset_;
  
  // Simulation flag if real hardware is not available
  bool use_dummy_mode_ = false;

  // --- 零点校准相关变量 ---
  CalibState calib_state_ = CalibState::INIT; // 默认为 INIT，上电即校准
  int calib_loop_count_ = 0;
  double calib_timer_ = 0.0; // 计时器
  std::vector<int32_t> calib_positions_;
  ec_sdo_request_t *sdo_home_offset_ = nullptr; // SDO 请求句柄
  
  // 用于 SDO 写入的辅助函数
  void write_sdo_int32(int32_t value);
  
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace ethercat_device_control

#endif  // ETHERCAT_DEVICE_CONTROL__ETHERCAT_HARDWARE_INTERFACE_HPP_
