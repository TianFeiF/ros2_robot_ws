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

// Zero Calibration State Machine
enum class CalibState {
  IDLE,               // Idle / Normal Operation
  INIT,               // Initialization
  LOOP_START,         // Start of a calibration loop
  MOVE_AWAY,          // Move away from hard stop (Ramping Up)
  MOVE_CLOSE,         // Move towards hard stop (Ramping Down/Negative)
  CALC_OFFSET,        // Calculate average offset
  MOVE_TO_ZERO,       // Move to zero position (Software)
  DONE                // Calibration Complete
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
  double current_calib_velocity_ = 0.0;
  
  // Zero calibration logic (Software Offset)
  void handle_zero_calibration(int slave_idx, const rclcpp::Duration & period, int8_t & target_mode, int16_t actual_torque);
  
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace ethercat_device_control

#endif  // ETHERCAT_DEVICE_CONTROL__ETHERCAT_HARDWARE_INTERFACE_HPP_
