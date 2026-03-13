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

// 辅助函数：写 SDO
void EthercatHardwareInterface::write_sdo_int32(int32_t value) {
    if (sdo_home_offset_) {
        // ecrt_sdo_request_data_size 是获取大小，不是设置大小
        // SDO 请求的大小在创建时已经指定为 4 字节
        
        // 使用 EC_WRITE_S32 处理字节序 (Host to Little Endian)
        EC_WRITE_S32(ecrt_sdo_request_data(sdo_home_offset_), value);
        // 触发写入请求
        ecrt_sdo_request_write(sdo_home_offset_);
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

    // --- 创建 SDO 请求 (0x607C:00) ---
    // Slave 0 (straight_joint) 
    if (slave_config_[0]) {
        // 创建 SDO 请求: 索引 0x607C, 子索引 0x00, 大小 4 bytes
        sdo_home_offset_ = ecrt_slave_config_create_sdo_request(slave_config_[0], 0x607C, 0x00, 4);
        if (!sdo_home_offset_) {
            RCLCPP_ERROR(rclcpp::get_logger("EthercatHardwareInterface"), "创建 SDO 请求 (0x607C) 失败！");
            return hardware_interface::CallbackReturn::ERROR;
        }
        // 设置超时
        ecrt_sdo_request_timeout(sdo_home_offset_, 2000); // 2000ms
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

  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "EtherCAT 系统开始运行。");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EthercatHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "EtherCAT 已停止!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!use_dummy_mode_) {
      ecrt_master_receive(master_);
      ecrt_domain_process(domain_);

      for (int i = 0; i < NUM_SLAVES; ++i) {
          // 默认操作模式为 CSP (8)
          int8_t target_mode = 8;
          uint16_t status = EC_READ_U16(domain_pd_ + pdo_offset_.status_word[i]);
          uint16_t control = 0x00;
          int16_t actual_torque = EC_READ_S16(domain_pd_ + pdo_offset_.actual_torque[i]);
          
          // --- 零点校准逻辑 (仅针对 straight_joint = Slave 0) ---
          if (i == 0 && calib_state_ != CalibState::DONE && calib_state_ != CalibState::IDLE) {
              
              switch (calib_state_) {
                  case CalibState::INIT:
                      RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "开始零点校准: 写入 0 到 0x607C (通过PDO)");
                      // 通过 PDO 写入 0
                      hw_home_offset_[i] = 0;
                      
                      // 为了确保写入生效，我们还是保留 SDO 读取检查
                      calib_state_ = CalibState::READ_ZERO;
                      break;

                  // 移除 WAIT_WRITE_ZERO 状态
                  
                  case CalibState::READ_ZERO:
                      // 等待几个周期让 PDO 生效
                      calib_timer_ += period.seconds();
                      if (calib_timer_ > 0.1) {
                           ecrt_sdo_request_read(sdo_home_offset_);
                           calib_state_ = CalibState::WAIT_READ_ZERO;
                           calib_timer_ = 0.0;
                      }
                      break;

                  case CalibState::WAIT_READ_ZERO:
                      if (ecrt_sdo_request_state(sdo_home_offset_) == EC_REQUEST_SUCCESS) {
                          // 确保使用 EC_READ_S32 处理字节序
                          int32_t val = EC_READ_S32(ecrt_sdo_request_data(sdo_home_offset_));
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "读取 0x607C 当前值: %d (应为0)", val);
                          calib_state_ = CalibState::LOOP_START;
                          calib_loop_count_ = 0;
                      } else if (ecrt_sdo_request_state(sdo_home_offset_) == EC_REQUEST_ERROR) {
                          RCLCPP_ERROR(rclcpp::get_logger("EthercatHardwareInterface"), "SDO 读取失败，重试...");
                          calib_state_ = CalibState::READ_ZERO;
                      }
                      break;

                  case CalibState::LOOP_START:
                      if (calib_loop_count_ < 3) {
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "校准循环 %d: 远离零点 (200力矩)", calib_loop_count_ + 1);
                          calib_timer_ = 0.0;
                          calib_state_ = CalibState::MOVE_AWAY;
                      } else {
                          calib_state_ = CalibState::CALC_OFFSET;
                      }
                      break;

                  case CalibState::MOVE_AWAY:
                      // 2. 进入同步力矩模式，电机以200的目标力矩运行2s
                      target_mode = 10; // CST
                      EC_WRITE_S16(domain_pd_ + pdo_offset_.target_torque[i], 200);
                      calib_timer_ += period.seconds();
                      if (calib_timer_ >= 2.0) {
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "校准循环 %d: 接近零点 (-300力矩)", calib_loop_count_ + 1);
                          calib_state_ = CalibState::MOVE_CLOSE;
                      }
                      break;

                  case CalibState::MOVE_CLOSE:
                      // 然后再以-300的目标力矩运行
                      target_mode = 10; // CST
                      EC_WRITE_S16(domain_pd_ + pdo_offset_.target_torque[i], -280);
                      
                      // 直到实际力矩小于-302后记录第一次实际位置
                      if (actual_torque < -282) {
                          int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[i]);
                          calib_positions_.push_back(current_pos);
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "记录位置: %d", current_pos);
                          
                          calib_loop_count_++;
                          calib_state_ = CalibState::LOOP_START;
                      }
                      break;

                  case CalibState::CALC_OFFSET:
                      {
                          // 3. 将三次的平均值取反并写入0x607C:00
                          double sum = std::accumulate(calib_positions_.begin(), calib_positions_.end(), 0.0);
                          double avg = sum / calib_positions_.size();
                          int32_t offset = static_cast<int32_t>(-avg);
                          
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "平均位置: %.2f, 写入偏移量: %d", avg, offset);
                          // 通过 PDO 写入偏移量
                          hw_home_offset_[i] = offset;
                          calib_state_ = CalibState::READ_OFFSET;
                          calib_timer_ = 0.0;
                      }
                      break;

                  case CalibState::READ_OFFSET:
                      calib_timer_ += period.seconds();
                      if (calib_timer_ > 0.1) {
                          ecrt_sdo_request_read(sdo_home_offset_);
                          calib_state_ = CalibState::WAIT_READ_OFFSET;
                          calib_timer_ = 0.0;
                      }
                      break;

                  case CalibState::WAIT_READ_OFFSET:
                      if (ecrt_sdo_request_state(sdo_home_offset_) == EC_REQUEST_SUCCESS) {
                          int32_t val = EC_READ_S32(ecrt_sdo_request_data(sdo_home_offset_));
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "读取 0x607C 新值: %d", val);
                          calib_timer_ = 0.0;
                          calib_state_ = CalibState::STABILIZE;
                      } else if (ecrt_sdo_request_state(sdo_home_offset_) == EC_REQUEST_ERROR) {
                          RCLCPP_ERROR(rclcpp::get_logger("EthercatHardwareInterface"), "SDO 读取失败，重试...");
                          calib_state_ = CalibState::READ_OFFSET;
                      }
                      break;
                  
                  case CalibState::STABILIZE:
                      // 直接失能电机，等待1s
                      // Controlword 将在下方被强制覆盖为 0x06
                      calib_timer_ += period.seconds();
                      if (calib_timer_ >= 1.0) {
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "系统稳定，准备运动到零点");
                          calib_state_ = CalibState::MOVE_TO_ZERO;
                      }
                      break;

                  case CalibState::MOVE_TO_ZERO:
                      {
                          // 再次读取当前真实位置
                           int32_t current_pos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[i]);
                           (void)current_pos; // 消除未使用变量警告
                          
                          // 由于我们希望运动到零点，且此时 Offset 已生效
                          // 我们应该将目标位置设为 0
                          // 但是为了安全，我们还是先同步当前位置，然后依靠上层控制器(MoveIt/Controller)发 0 指令
                          // 或者，如果这是一个单纯的 Homing 过程，我们可以在这里强制设为 0
                          
                          // 根据用户要求：最后应该运动到 0 点
                          EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[i], 0);
                          hw_commands_position_[i] = 0.0;
                          
                          RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "零点校准完全完成，强制目标位置为 0，切换回 CSP 模式");
                          calib_state_ = CalibState::DONE;
                      }
                      break;
                      
                  default:
                      break;
              }
          }

          // 1. 设置操作模式 (CST during calibration, CSP otherwise)
          EC_WRITE_S8(domain_pd_ + pdo_offset_.mode_of_operation[i], target_mode);

          // 2. 状态机逻辑
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
                          
                          // 特殊处理：如果刚刚完成校准 (DONE)，目标位置应该为 0
                          if (i == 0 && calib_state_ == CalibState::DONE) {
                              EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[i], 0);
                          } else {
                              EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[i], current_pos);
                          }
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

              // 强制失能 (STABILIZE 阶段)
              if (i == 0 && calib_state_ == CalibState::STABILIZE) {
                  control = 0x06;
              }
              
              // 强制状态转换 (MOVE_TO_ZERO 阶段): 确保从 0x06 -> 0x07 -> 0x0F
              // 因为在 STABILIZE 阶段我们强制了 0x06 (Shutdown)
              // 当进入 MOVE_TO_ZERO -> DONE 后，状态机需要重新使能
              // 这里我们不做特殊处理，依赖上面的标准状态机逻辑即可
              // 但是为了防止 DONE 瞬间的跳变，我们在 MOVE_TO_ZERO 中已经设置了 Target=0
          }
          EC_WRITE_U16(domain_pd_ + pdo_offset_.control_word[i], control);
      }
      
      // 更新 ROS 2 Control 的状态变量
      for (uint i = 0; i < info_.joints.size(); i++)
      {
          std::string joint_name = info_.joints[i].name;
          
          if (JOINT_TO_SLAVE_MAP.count(joint_name) > 0) {
              int slave_idx = JOINT_TO_SLAVE_MAP.at(joint_name);
              
              int32_t pos_val = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[slave_idx]);
              hw_states_position_[i] = static_cast<double>(pos_val);
              
              // 读取速度和力矩
              int32_t vel_val = EC_READ_S32(domain_pd_ + pdo_offset_.actual_velocity[slave_idx]);
              hw_states_velocity_[i] = static_cast<double>(vel_val);
              
              int16_t tor_val = EC_READ_S16(domain_pd_ + pdo_offset_.actual_torque[slave_idx]);
              hw_states_effort_[i] = static_cast<double>(tor_val);

              // 关键：如果还没 Enabled 或者 正在校准，强制 Command = State
                          uint16_t status = EC_READ_U16(domain_pd_ + pdo_offset_.status_word[slave_idx]);
                          bool is_calibrating = (slave_idx == 0 && calib_state_ != CalibState::DONE && calib_state_ != CalibState::IDLE);
                          
                          if ((status & 0x6F) != 0x27 || is_calibrating) { 
                              // 注意：如果刚刚完成校准 (DONE)，我们不应该在这里覆盖 hw_commands_position_
                              // 因为我们在 MOVE_TO_ZERO 中已经将其设为 0 了
                              // 但是，如果 ROS Controller 还没启动，hw_commands_position_ 可能会被 Controller 覆盖
                              
                              // 这里我们只在非 DONE 状态下同步
                              if (calib_state_ != CalibState::DONE) {
                                  hw_commands_position_[i] = hw_states_position_[i];
                              }
                          }

                          // 调试日志：监控状态字变化
                          static uint16_t last_status = 0;
                          if (slave_idx == 0 && status != last_status) {
                              RCLCPP_INFO(rclcpp::get_logger("EthercatHardwareInterface"), "Slave 0 Status Changed: 0x%04X", status);
                              last_status = status;
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
              
              // 只有当状态为 Operation Enabled 且 不在校准时，才更新目标位置
              uint16_t status = EC_READ_U16(domain_pd_ + pdo_offset_.status_word[slave_idx]);
              bool is_calibrating = (slave_idx == 0 && calib_state_ != CalibState::DONE && calib_state_ != CalibState::IDLE);

              if ((status & 0x6F) == 0x27 && !is_calibrating) {
                   
                   if (slave_idx == 0 && calib_state_ == CalibState::DONE) {
                       // 刚刚完成校准，我们可能还没收到来自 Controller 的 0 指令
                       // 此时 hw_commands_position_ 可能还是旧值 (校准前的状态)
                       // 强制覆盖为 0
                       if (std::abs(hw_commands_position_[i]) > 100000) { // 简单保护：如果指令位置太大，说明 Controller 还没接管
                           hw_commands_position_[i] = 0;
                       }
                   }
                   
                   EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[slave_idx], static_cast<int32_t>(hw_commands_position_[i]));
                   if(slave_idx == 0)
                   {
                      // 写入 home offset 到 PDO
                      EC_WRITE_S32(domain_pd_ + pdo_offset_.home_offset[slave_idx], hw_home_offset_[slave_idx]);
                      int32_t actpos = EC_READ_S32(domain_pd_ + pdo_offset_.actual_position[slave_idx]);
                      int32_t tagpos = EC_READ_S32(domain_pd_ + pdo_offset_.target_position[slave_idx]);
                      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("EthercatHardwareInterface"), *clock_, 1000, "轴: %d, 实际位置: %d , 目标位置： %d, 状态字: 0x%04X", slave_idx, actpos, tagpos, status);
                   }
                   
              } else if (slave_idx == 0) {
                  // 即使在校准状态，也需要写入 home offset
                  EC_WRITE_S32(domain_pd_ + pdo_offset_.home_offset[slave_idx], hw_home_offset_[slave_idx]);
                  
                  // 强制写入目标位置为 0 (仅当处于 DONE 状态且刚切换回 CSP 时)
                  if (calib_state_ == CalibState::DONE && (status & 0x6F) != 0x27) {
                      EC_WRITE_S32(domain_pd_ + pdo_offset_.target_position[slave_idx], 0);
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
