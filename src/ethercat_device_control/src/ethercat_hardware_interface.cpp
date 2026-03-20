#include "ethercat_device_control/ethercat_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <thread> // For std::this_thread::sleep_for
#include <numeric> // For std::accumulate

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// 引入 EtherCAT 主站库 ecrt.h
#include <ecrt.h>

// C++ 命名空间
namespace ethercat_device_control
{

// --- EtherCAT 常量定义 ---
const int NUM_SLAVES = 5; // 从站数量
const int EYOU_SERVO_ALIAS = 0; // 别名地址，通常为0
// 从站的位置（总线上的物理顺序）
const uint16_t EYOU_SERVO_POS[NUM_SLAVES] = {0, 1, 2, 3, 4};

const uint32_t VENDOR_ID = 0x00001097;    // 厂商ID (根据 ethercat cstruct 输出)
const uint32_t PRODUCT_CODE = 0x00002406; // 产品代码 (根据 ethercat cstruct 输出)

// 转换常量 (straight_joint)
// 1 圈 = 65535 * 101 pulses
// 1 圈 = 20 mm = 0.02 m
// Pulses/Meter = (65535 * 101) / 0.02 = 330951750
const double STRAIGHT_JOINT_PULSES_PER_METER = (65535.0 * 101.0) / 0.02;

// 转换常量 (wheel_joint)
const double WHEEL_JOINT_PULSES_PER_REV = 65535.0 * 101.0;
const double WHEEL_JOINT_RAD_PER_REV = 2.0 * M_PI;
const double WHEEL_JOINT_PULSES_PER_RAD = WHEEL_JOINT_PULSES_PER_REV / WHEEL_JOINT_RAD_PER_REV;

// 转换常量 (clamp_wheel_joint)
const double CLAMP_WHEEL_TORQUE_RAW_MAX = 1000.0;
const double CLAMP_WHEEL_TORQUE_NM_MAX = 6.8;
const double CLAMP_WHEEL_NM_PER_RAW = CLAMP_WHEEL_TORQUE_NM_MAX / CLAMP_WHEEL_TORQUE_RAW_MAX;

// CiA 402 Controlword (控制字) 命令定义
#define CTRL_SHUTDOWN       0x0006 // 关闭
#define CTRL_SWITCH_ON      0x0007 // 开启
#define CTRL_ENABLE_OP      0x000F // 启用操作
#define CTRL_DISABLE_VOLT   0x0000 // 禁用电压
#define CTRL_RESET_FAULT    0x0080 // 复位故障 (Bit 7)

// --- 基于 `main.cpp` 示例的配置 ---
// 必须严格匹配示例代码中的 PDO 结构

// PDO 条目定义 (Entries)
ec_pdo_entry_info_t slave_pdo_entries[] = {
    // RxPDO (Master -> Slave) - 索引 0-5
    {0x6040, 0x00, 16}, // Controlword
    {0x607A, 0x00, 32}, // Target position
    {0x60FF, 0x00, 32}, // Target velocity
    {0x6071, 0x00, 16}, // Target torque
    {0x6060, 0x00, 8},  // Modes of operation
    {0x607C, 0x00, 32}, // Home offset
    {0x60C2, 0x01, 8},  // Redundant config (alignment)

    // TxPDO (Slave -> Master) - 索引 7-13
    {0x6041, 0x00, 16}, // Statusword
    {0x6064, 0x00, 32}, // Position actual value
    {0x606C, 0x00, 32}, // Velocity actual value
    {0x6077, 0x00, 16}, // Torque actual value
    {0x6061, 0x00, 8},  // Modes of operation display
    {0x603F, 0x00, 16}, // Error code
    {0x2026, 0x00, 8},  // Reserved
};

// PDO 配置 (PDOs)
ec_pdo_info_t slave_rx_pdo[] = {
    {0x1600, 7, slave_pdo_entries + 0}, /* RxPDO 包含前7个条目 */
};

ec_pdo_info_t slave_tx_pdo[] = {
    {0x1A00, 7, slave_pdo_entries + 7}, /* TxPDO 包含后7个条目 */
};

// Sync Manager 配置 (Syncs)
ec_sync_info_t slave_syncs[] = {
    // Index, Direction, Watchdog Mode, PDOs, Watchdog Enable
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_rx_pdo, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_tx_pdo, EC_WD_DISABLE},
    {0xff}
};

// 关节名称到从站索引的映射表
const std::map<std::string, int> JOINT_TO_SLAVE_MAP = {
    {"straight_joint", 0},
    {"front_wheel_joint", 1},
    {"rear_wheel_joint", 2},
    {"rear_clamp_wheel_joint", 3},
    {"front_clamp_wheel_joint", 4}
};

// Zero Calibration Logic (Slave 0 Only)
void EthercatHardwareInterface::handle_zero_calibration(int slave_idx, const rclcpp::Duration & period, int8_t & target_mode, int16_t actual_torque) {
    if (slave_idx != 0) return; // Only for straight_joint (Slave 0)
    if (calib_state_ == CalibState::DONE || calib_state_ == CalibState::IDLE) return;

    switch (calib_state_) {
        case CalibState::INIT:
            RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "Start Zero Calibration: Initialize Software Offset");
            hw_home_offset_[slave_idx] = 0;
            calib_state_ = CalibState::LOOP_START;
            calib_loop_count_ = 0;
            break;

        case CalibState::LOOP_START:
            if (calib_loop_count_ < 3) {
                RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "Calibration Loop %d: Move Away (Ramping Up)", calib_loop_count_ + 1);
                calib_timer_ = 0.0;
                current_calib_velocity_ = 0.0;
                calib_state_ = CalibState::MOVE_AWAY;
            } else {
                calib_state_ = CalibState::CALC_OFFSET;
            }
            break;

        case CalibState::MOVE_AWAY:
            // 1. Enter CSV Mode
            target_mode = 9; // CSV
            
            // 2. Ramping Up Velocity (Target: 50000 pulses/s)
            if (current_calib_velocity_ < 1103172*2) {
                current_calib_velocity_ += 600000.0 * period.seconds();
                if (current_calib_velocity_ > 1103172*2) current_calib_velocity_ = 1103172*2;
            }
            
            EC_WRITE_S32(domain_pd_ + pdo_offset_.target_velocity[slave_idx], (int32_t)current_calib_velocity_);

            // 3. Run for 2.0 seconds
            calib_timer_ += period.seconds();
            if (calib_timer_ >= 2.0) {
                RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "Calibration Loop %d: Move Close (Ramping Down)", calib_loop_count_ + 1);
                calib_state_ = CalibState::MOVE_CLOSE;
                calib_timer_ = 0.0; // Reset timer for next state
            }
            break;

        case CalibState::MOVE_CLOSE:
            // 1. Enter CSV Mode
            target_mode = 9; // CSV
            
            // 2. Ramping Down to Negative Velocity (Target: -50000 pulses/s)
            if (current_calib_velocity_ > -1103172) {
                current_calib_velocity_ -= 600000.0 * period.seconds();
                if (current_calib_velocity_ < -1103172) current_calib_velocity_ = -1103172;
            }
            
            EC_WRITE_S32(domain_pd_ + pdo_offset_.target_velocity[slave_idx], (int32_t)current_calib_velocity_);
            
            calib_timer_ += period.seconds();
            
            // 3. Detect Hard Stop (Torque Limit)
            // Use absolute value for robustness
            if (std::abs(actual_torque) > 250) {
            //if (actual_torque < -250) {
                int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[slave_idx]);
                calib_positions_.push_back(current_pos);
                RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "Hard Stop Detected (Torque: %d), Position: %d", actual_torque, current_pos);
                
                calib_loop_count_++;
                current_calib_velocity_ = 0.0; // Reset velocity
                calib_state_ = CalibState::LOOP_START;
            } else if (calib_timer_ > 60.0) { // Timeout Protection (10s)
                RCLCPP_WARN(rclcpp::get_logger("EthercatHardwareInterface"), "Calibration Timeout (%.1f s), Retrying Loop...", calib_timer_);
                current_calib_velocity_ = 0.0;
                calib_state_ = CalibState::LOOP_START;
            }
            break;

        case CalibState::CALC_OFFSET:
            {
                // Calculate Average Position as Software Offset
                double sum = std::accumulate(calib_positions_.begin(), calib_positions_.end(), 0.0);
                double avg = sum / calib_positions_.size();
                int32_t offset = static_cast<int32_t>(avg);
                
                RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "Avg Position: %.2f, Software Offset: %d", avg, offset);
                // Save offset to member variable (applied in read/write)
                hw_home_offset_[slave_idx] = offset;
                
                calib_state_ = CalibState::MOVE_TO_ZERO;
                calib_timer_ = 0.0;
            }
            break;

        case CalibState::MOVE_TO_ZERO:
            {
                // Read current hardware position
                int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[slave_idx]);
                
                // Set target to current position to prevent jump
                EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[slave_idx], current_pos);
                
                // Update ROS 2 Control State (Software Position = Hardware - Offset)
                int32_t software_pos = current_pos - hw_home_offset_[slave_idx];
                hw_commands_position_[slave_idx] = static_cast<double>(software_pos) / STRAIGHT_JOINT_PULSES_PER_METER;
                
                RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "Calibration Done. Hardware Pos: %d, Software Pos: %d", current_pos, software_pos);
                calib_state_ = CalibState::DONE;
            }
            break;
            
        default:
            break;
    }
}

// 硬件接口初始化函数
hardware_interface::CallbackReturn EthercatHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "初始化 EtherCAT 硬件接口...");

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  
  hw_commands_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const auto& pair : JOINT_TO_SLAVE_MAP) {
      bool found = false;
      for (const auto& joint : info_.joints) {
          if (joint.name == pair.first) {
              found = true;
              RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), 
                  "关节 '%s' 成功映射到 EtherCAT Slave %d", pair.first.c_str(), pair.second);
              break;
          }
      }
      if (!found) {
          RCLCPP_WARN(
            rclcpp::get_logger("EthercatHardwareInterface"),
            "EtherCAT 映射表中的关节 '%s' 未在 URDF (ros2_control tag) 中找到！", pair.first.c_str());
      }
  }

  // Resize vectors for all slaves
  slave_config_.resize(NUM_SLAVES, nullptr);
  
  // RxPDO offsets
  pdo_offset_.control_word.resize(NUM_SLAVES);
  pdo_offset_.target_position.resize(NUM_SLAVES);
  pdo_offset_.target_velocity.resize(NUM_SLAVES);
  pdo_offset_.target_torque.resize(NUM_SLAVES);
  pdo_offset_.mode_of_operation.resize(NUM_SLAVES);
  pdo_offset_.dummy_byte_1.resize(NUM_SLAVES);
  
  // TxPDO offsets
  pdo_offset_.status_word.resize(NUM_SLAVES);
  pdo_offset_.actual_position.resize(NUM_SLAVES);
  pdo_offset_.actual_velocity.resize(NUM_SLAVES);
  pdo_offset_.actual_torque.resize(NUM_SLAVES);
  pdo_offset_.mode_of_operation_display.resize(NUM_SLAVES);
  pdo_offset_.error_code.resize(NUM_SLAVES);
  pdo_offset_.dummy_byte_2.resize(NUM_SLAVES);

  if (info_.hardware_parameters.count("use_dummy_mode") > 0) {
    if (info_.hardware_parameters.at("use_dummy_mode") == "true") {
      use_dummy_mode_ = true;
      RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "使用模拟模式 (Dummy Mode)");
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// 配置函数
hardware_interface::CallbackReturn EthercatHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "正在配置 EtherCAT 主站...");

  if (!use_dummy_mode_) {
    master_ = ecrt_request_master(0);
    if (!master_) {
        RCLCPP_FATAL(rclcpp::get_logger("EthercatHardwareInterface"), "请求主站失败！");
        return hardware_interface::CallbackReturn::ERROR;
    }

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        RCLCPP_FATAL(rclcpp::get_logger("EthercatHardwareInterface"), "创建域失败");
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (int i = 0; i < NUM_SLAVES; ++i) {
        slave_config_[i] = ecrt_master_slave_config(master_, EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE);
        if (!slave_config_[i]) {
            RCLCPP_FATAL(rclcpp::get_logger("EthercatHardwareInterface"), "获取从站 %d 配置失败", i);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (ecrt_slave_config_pdos(slave_config_[i], EC_END, slave_syncs)) {
            RCLCPP_FATAL(rclcpp::get_logger("EthercatHardwareInterface"), "配置从站 %d 的 PDO 失败", i);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

  // 预分配所有 PDO offset 向量
  pdo_offset_.control_word.resize(NUM_SLAVES);
  pdo_offset_.target_position.resize(NUM_SLAVES);
  pdo_offset_.target_velocity.resize(NUM_SLAVES);
  pdo_offset_.target_torque.resize(NUM_SLAVES);
  pdo_offset_.mode_of_operation.resize(NUM_SLAVES);
  pdo_offset_.home_offset.resize(NUM_SLAVES);
  pdo_offset_.dummy_byte_1.resize(NUM_SLAVES);
  
  pdo_offset_.status_word.resize(NUM_SLAVES);
  pdo_offset_.actual_position.resize(NUM_SLAVES);
  pdo_offset_.actual_velocity.resize(NUM_SLAVES);
  pdo_offset_.actual_torque.resize(NUM_SLAVES);
  pdo_offset_.mode_of_operation_display.resize(NUM_SLAVES);
  pdo_offset_.error_code.resize(NUM_SLAVES);
  pdo_offset_.dummy_byte_2.resize(NUM_SLAVES);

  // 注册 PDO 条目到域 (14 entries per slave)
  ec_pdo_entry_reg_t domain_regs[NUM_SLAVES * 14 + 1];
    int reg_idx = 0;
    
    for (int i = 0; i < NUM_SLAVES; ++i) {
        // RxPDO
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x6040, 0, &pdo_offset_.control_word[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x607A, 0, &pdo_offset_.target_position[i], NULL}; 
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x60FF, 0, &pdo_offset_.target_velocity[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x6071, 0, &pdo_offset_.target_torque[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x6060, 0, &pdo_offset_.mode_of_operation[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x607C, 0, &pdo_offset_.home_offset[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x60C2, 1, &pdo_offset_.dummy_byte_1[i], NULL}; // subindex 1
        
        // TxPDO
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x6041, 0, &pdo_offset_.status_word[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x6064, 0, &pdo_offset_.actual_position[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x606C, 0, &pdo_offset_.actual_velocity[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x6077, 0, &pdo_offset_.actual_torque[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x6061, 0, &pdo_offset_.mode_of_operation_display[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x603F, 0, &pdo_offset_.error_code[i], NULL};
        domain_regs[reg_idx++] = {EYOU_SERVO_ALIAS, EYOU_SERVO_POS[i], VENDOR_ID, PRODUCT_CODE, 0x2026, 0, &pdo_offset_.dummy_byte_2[i], NULL};
    }
    domain_regs[reg_idx] = {};

    if (ecrt_domain_reg_pdo_entry_list(domain_, domain_regs)) {
        RCLCPP_FATAL(rclcpp::get_logger("EthercatHardwareInterface"), "注册 PDO 列表失败");
        return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 初始化时钟
  clock_ = rclcpp::Clock::make_shared();

  // 初始化命令和状态
  hw_home_offset_.resize(info_.joints.size(), 0);
  for (uint i = 0; i < info_.joints.size(); i++) {
      hw_states_position_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_states_effort_[i] = 0;
      hw_commands_position_[i] = 0;
      hw_commands_velocity_[i] = 0;
      hw_commands_effort_[i] = 0;
      hw_home_offset_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "EtherCAT 配置成功!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EthercatHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_states_effort_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EthercatHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
  }
  return command_interfaces;
}

// 激活函数
hardware_interface::CallbackReturn EthercatHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "正在激活 EtherCAT 主站...");

  if (!use_dummy_mode_) {
    if (ecrt_master_activate(master_)) {
        RCLCPP_FATAL(rclcpp::get_logger("EthercatHardwareInterface"), "主站激活失败");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_) {
        RCLCPP_FATAL(rclcpp::get_logger("EthercatHardwareInterface"), "获取域数据指针失败");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // --- 等待所有从站进入 OP 状态 (参考 main.cpp) ---
    RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "等待从站进入 OP 模式...");
    
    // 尝试最多 5000 次，每次间隔 1ms (共 5秒)
    for (int i = 0; i < 5000; i++) {
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);

        if (i % 100 == 0) { // 每 100ms 检查一次
            int op_count = 0;
            for (int j = 0; j < NUM_SLAVES; j++) {
                ec_slave_config_state_t s;
                ecrt_slave_config_state(slave_config_[j], &s);
                if (s.al_state == EC_AL_STATE_OP) {
                    op_count++;
                }
            }
            if (op_count == NUM_SLAVES) {
                RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "所有从站已进入 OP 模式!");
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // 初始化校准状态
  calib_state_ = CalibState::INIT;
  calib_loop_count_ = 0;
  calib_timer_ = 0.0;
  calib_positions_.clear();
  
  if (!use_dummy_mode_) {
      RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "开始阻塞式零点校准...");
      auto last_time = std::chrono::steady_clock::now();
      
      while (rclcpp::ok() && calib_state_ != CalibState::DONE) {
           auto now = std::chrono::steady_clock::now();
           auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - last_time);
           rclcpp::Duration period(period_ns);
           last_time = now;
           
          ecrt_master_receive(master_);
          ecrt_domain_process(domain_);
          
          for (int i = 0; i < NUM_SLAVES; ++i) {
              // 默认 CSP
              int8_t target_mode = 8;
              if (i == 1 || i == 2) target_mode = 9; // CSV
              else if (i == 3 || i == 4) target_mode = 10; // CST
              
              int16_t actual_torque = EC_READ_S16(domain_pd_ + pdo_offset_.actual_torque[i]);
              
              // 调用校准逻辑 (仅 Slave 0)
              handle_zero_calibration(i, period, target_mode, actual_torque);
              
              // 写入 Mode of Operation
              EC_WRITE_S8(domain_pd_ + pdo_offset_.mode_of_operation[i], target_mode);
              
              // 状态机逻辑 (Enable Operation)
              uint16_t status = EC_READ_U16(domain_pd_ + pdo_offset_.status_word[i]);
              uint16_t control = 0x00;
              
              if (status & (1 << 3)) { // Fault
                  control = 0x80; // Fault Reset
              } else {
                  switch (status & 0x6F) {
                      case 0x00:
                      case 0x40:
                          control = 0x06; // Shutdown
                          break;
                      case 0x21: // Ready to Switch On
                          control = 0x07; // Switch On
                          {
                              // 防止飞车: 确保在使能前，目标位置等于当前位置
                              int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[i]);
                              EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[i], current_pos);
                          }
                          break;
                      case 0x23: // Switched On
                          control = 0x0F; // Enable Operation
                          break;
                      case 0x27: // Operation Enabled
                          control = 0x0F; // Enable Operation
                          break;
                      default:
                          control = 0x06; // Shutdown (Default)
                          break;
                  }
              }
              EC_WRITE_U16(domain_pd_ + pdo_offset_.control_word[i], control);
              
              // Note: Offset writing is handled in software now, no need to write to hardware 0x607C
          }
          
          ecrt_domain_queue(domain_);
          ecrt_master_send(master_);
          
          // 维持 1ms 周期
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      if (!rclcpp::ok()) {
          RCLCPP_WARN(rclcpp::get_logger("EthercatHardwareInterface"), "阻塞式零点校准被中断！正在停止电机...");
          // 循环几次确保命令下达并让状态机响应
          for (int cycle = 0; cycle < 50; ++cycle) {
              ecrt_master_receive(master_);
              ecrt_domain_process(domain_);

              for (int i = 0; i < NUM_SLAVES; ++i) {
                  // 1. 将目标速度和力矩设为 0
                  EC_WRITE_S32(domain_pd_ + pdo_offset_.target_velocity[i], 0);
                  EC_WRITE_S16(domain_pd_ + pdo_offset_.target_torque[i], 0);

                  // 2. 将目标位置设为当前位置，防止跳变
                  int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[i]);
                  EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[i], current_pos);

                  // 3. 将控制字设为 Shutdown (0x06)，使电机退出 Operation Enabled 状态
                  EC_WRITE_U16(domain_pd_ + pdo_offset_.control_word[i], CTRL_SHUTDOWN);
              }

              ecrt_domain_queue(domain_);
              ecrt_master_send(master_);

              std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "中断处理完成，电机已安全关闭!");
          return hardware_interface::CallbackReturn::ERROR;
      }

      RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "阻塞式零点校准完成!");
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "EtherCAT 系统开始运行。");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EthercatHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "正在停止 EtherCAT 电机并关闭...");

  if (!use_dummy_mode_ && master_ && domain_pd_) {
      // 循环几次确保命令下达并让状态机响应
      for (int cycle = 0; cycle < 50; ++cycle) {
          ecrt_master_receive(master_);
          ecrt_domain_process(domain_);

          for (int i = 0; i < NUM_SLAVES; ++i) {
              // 1. 将目标速度和力矩设为 0
              EC_WRITE_S32(domain_pd_ + pdo_offset_.target_velocity[i], 0);
              EC_WRITE_S16(domain_pd_ + pdo_offset_.target_torque[i], 0);

              // 2. 将目标位置设为当前位置，防止跳变
              int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[i]);
              EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[i], current_pos);

              // 3. 将控制字设为 Shutdown (0x06)，使电机退出 Operation Enabled 状态
              EC_WRITE_U16(domain_pd_ + pdo_offset_.control_word[i], CTRL_SHUTDOWN);
          }

          ecrt_domain_queue(domain_);
          ecrt_master_send(master_);

          std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "EtherCAT 已停止，电机已安全关闭!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!use_dummy_mode_) {
      ecrt_master_receive(master_);
      ecrt_domain_process(domain_);

      for (int i = 0; i < NUM_SLAVES; ++i) {
          // Default to CSP Mode (8)
          int8_t target_mode = 8;
          if (i == 1 || i == 2) {
              target_mode = 9; // CSV for Wheels
          } else if (i == 3 || i == 4) {
              target_mode = 10; // CST for Clamp Wheels
          }
          
          uint16_t status = EC_READ_U16(domain_pd_ + pdo_offset_.status_word[i]);
          uint16_t control = 0x00;
          
          // Note: Actual torque is read but unused in main loop logic (used in calibration)
          // int16_t actual_torque = EC_READ_S16(domain_pd_ + pdo_offset_.actual_torque[i]);

          // 1. Set Operation Mode
          EC_WRITE_S8(domain_pd_ + pdo_offset_.mode_of_operation[i], target_mode);
          
          // 2. Standard CiA 402 State Machine
          if (status & (1 << 3)) { // Fault
              control = 0x80; // Fault Reset
          } else {
              switch (status & 0x6F) {
                  case 0x00:
                  case 0x40:
                      control = 0x06; // Shutdown
                      break;
                  case 0x21: // Ready to Switch On
                      control = 0x07; // Switch On
                      {
                          // Prevent Jump: Sync Target to Actual before enabling
                          int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[i]);
                          EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[i], current_pos);
                      }
                      break;
                  case 0x23: // Switched On
                      control = 0x0F; // Enable Operation
                      break;
                  case 0x27: // Operation Enabled
                      control = 0x0F; // Enable Operation
                      break;
                  default:
                      control = 0x06; // Shutdown (Default)
                      break;
              }
          }
          EC_WRITE_U16(domain_pd_ + pdo_offset_.control_word[i], control);
      }
      
      // Update ROS 2 Control State Variables
      for (uint i = 0; i < info_.joints.size(); i++)
      {
          std::string joint_name = info_.joints[i].name;
          
          if (JOINT_TO_SLAVE_MAP.count(joint_name) > 0) {
              int slave_idx = JOINT_TO_SLAVE_MAP.at(joint_name);
              
              // Read Position
              int32_t pos_val = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[slave_idx]);
              if (slave_idx == 0) {
                  // Slave 0 (straight_joint): Apply Software Offset
                  int32_t software_pos = pos_val - hw_home_offset_[slave_idx];
                  hw_states_position_[i] = static_cast<double>(software_pos) / STRAIGHT_JOINT_PULSES_PER_METER;
              } else if (slave_idx == 1 || slave_idx == 2) {
                  // Slave 1, 2 (wheel_joints): Convert pulses to radians
                  hw_states_position_[i] = static_cast<double>(pos_val) / WHEEL_JOINT_PULSES_PER_RAD;
              } else {
                  hw_states_position_[i] = static_cast<double>(pos_val) / WHEEL_JOINT_PULSES_PER_RAD;
              }
              
              // Read Velocity
              int32_t vel_val = EC_READ_S32(domain_pd_ + pdo_offset_.actual_velocity[slave_idx]);
              if (slave_idx == 0) {
                   hw_states_velocity_[i] = static_cast<double>(vel_val) / STRAIGHT_JOINT_PULSES_PER_METER;
              } else if (slave_idx == 1 || slave_idx == 2) {
                   // Slave 1, 2 (wheel_joints): Convert pulses/s to rad/s
                   hw_states_velocity_[i] = static_cast<double>(vel_val) / WHEEL_JOINT_PULSES_PER_RAD;
              } else {
                   hw_states_velocity_[i] = static_cast<double>(vel_val) / WHEEL_JOINT_PULSES_PER_RAD;
              }
              
              // Read Torque
              int16_t tor_val = EC_READ_S16(domain_pd_ + pdo_offset_.actual_torque[slave_idx]);
              if (slave_idx == 3 || slave_idx == 4) {
                   // Slave 3, 4 (clamp_wheel_joints): Convert raw to Nm
                   hw_states_effort_[i] = static_cast<double>(tor_val) * CLAMP_WHEEL_NM_PER_RAW;
              } else {
                   hw_states_effort_[i] = static_cast<double>(tor_val);
              }

              // Sync Command with State if not Operation Enabled
              // This prevents jumps when controller starts
              uint16_t status = EC_READ_U16(domain_pd_ + pdo_offset_.status_word[slave_idx]);
              if ((status & 0x6F) != 0x27) { 
                  hw_commands_position_[i] = hw_states_position_[i];
              }

          } else {
              // Loopback for unmapped joints
              hw_states_position_[i] = hw_commands_position_[i];
              hw_states_velocity_[i] = hw_commands_velocity_[i];
              hw_states_effort_[i] = hw_commands_effort_[i];
          }
      }
  } else {
      // Dummy Mode
      for (uint i = 0; i < info_.joints.size(); i++) {
          hw_states_position_[i] = hw_commands_position_[i];
          hw_states_velocity_[i] = hw_commands_velocity_[i];
          hw_states_effort_[i] = hw_commands_effort_[i];
      }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!use_dummy_mode_) {
      for (uint i = 0; i < info_.joints.size(); i++)
      {
          std::string joint_name = info_.joints[i].name;
          
          if (JOINT_TO_SLAVE_MAP.count(joint_name) > 0) {
              int slave_idx = JOINT_TO_SLAVE_MAP.at(joint_name);
              
              // Only update target when Operation Enabled and Not Calibrating
              uint16_t status = EC_READ_U16(domain_pd_ + pdo_offset_.status_word[slave_idx]);
              bool is_calibrating = (slave_idx == 0 && calib_state_ != CalibState::DONE && calib_state_ != CalibState::IDLE);

              if ((status & 0x6F) == 0x27 && !is_calibrating) {
                   
                   if (slave_idx == 0) {
                       // Slave 0: Meters -> Pulses, Apply Software Offset
                       // Target Hardware = (Target Software * Scale) + Hardware Offset
                       int32_t target_software_pulses = static_cast<int32_t>(hw_commands_position_[i] * STRAIGHT_JOINT_PULSES_PER_METER);
                       int32_t target_hardware_pulses = target_software_pulses + hw_home_offset_[slave_idx];
                       //RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"),"Target Software Pulses: %d, Target Hardware Pulses: %d", target_software_pulses, target_hardware_pulses);
                       EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[slave_idx], target_hardware_pulses);
                   } else if (slave_idx == 1 || slave_idx == 2) {
                       // Slave 1, 2 (wheel_joints): rad/s -> pulses/s
                       int32_t target_hardware_vel = static_cast<int32_t>(hw_commands_velocity_[i] * WHEEL_JOINT_PULSES_PER_RAD);
                       EC_WRITE_S32(domain_pd_ + pdo_offset_.target_velocity[slave_idx], target_hardware_vel);
                   } else if (slave_idx == 3 || slave_idx == 4) {
                       // Slave 3, 4 (clamp_wheel_joints): Nm -> raw
                       int16_t target_hardware_torque = static_cast<int16_t>(hw_commands_effort_[i] / CLAMP_WHEEL_NM_PER_RAW);
                       EC_WRITE_S16(domain_pd_ + pdo_offset_.target_torque[slave_idx], target_hardware_torque);
                   }
                   
              }
          }
      }

      ecrt_domain_queue(domain_);
      ecrt_master_send(master_);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ethercat_device_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_device_control::EthercatHardwareInterface, hardware_interface::SystemInterface)
