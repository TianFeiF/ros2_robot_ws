#include "rm_hardware_interface/rm_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>

namespace rm_hardware_interface
{

hardware_interface::CallbackReturn RmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize storage
  hw_states_position_.resize(info_.joints.size(), 0.0);
  hw_states_velocity_.resize(info_.joints.size(), 0.0);
  hw_commands_position_.resize(info_.joints.size(), 0.0);
  hw_commands_velocity_.resize(info_.joints.size(), 0.0);

  // Identify joints
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    std::string name = info_.joints[i].name;
    joint_name_to_index_[name] = i;
    
    // Simple heuristic: if name contains "grep", it's a gripper
    if (name.find("grep") != std::string::npos) {
        gripper_joint_index_ = i;
        has_gripper_ = true;
        gripper_joint_name_ = name;
        RCLCPP_INFO(rclcpp::get_logger("RmHardwareInterface"), "Identified gripper joint: %s", name.c_str());
    } else {
        arm_joint_names_.push_back(name);
        RCLCPP_INFO(rclcpp::get_logger("RmHardwareInterface"), "Identified arm joint: %s", name.c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create node for communication
  // Use a unique name to avoid conflict
  node_ = std::make_shared<rclcpp::Node>("rm_hardware_interface_node");

  // Subscribers
  // Note: joint_states comes from rm_driver, which publishes names "joint1", "joint2"...
  // We need to handle name mapping if they differ from "joint_1", "joint_2"...
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/rm_joint_states", 10,
    std::bind(&RmHardwareInterface::joint_state_callback, this, std::placeholders::_1));

  if (has_gripper_) {
      hand_status_sub_ = node_->create_subscription<rm_ros_interfaces::msg::Handstatus>(
        "/rm_driver/udp_hand_status", 10,
        std::bind(&RmHardwareInterface::hand_status_callback, this, std::placeholders::_1));
        
      gripper_pub_ = node_->create_publisher<rm_ros_interfaces::msg::Gripperset>(
        "/rm_driver/set_gripper_position_cmd", 10);
  }

  // Publishers
  movej_pub_ = node_->create_publisher<rm_ros_interfaces::msg::Jointpos>(
    "/rm_driver/movej_canfd_cmd", 10);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_position_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset commands to current states to avoid jumps
  for (size_t i = 0; i < hw_states_position_.size(); ++i) {
      if (std::isnan(hw_states_position_[i])) {
          hw_commands_position_[i] = 0.0;
      } else {
          hw_commands_position_[i] = hw_states_position_[i];
      }
      hw_commands_velocity_[i] = 0.0;
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Process callbacks
  if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. Publish Arm Commands
  auto msg = rm_ros_interfaces::msg::Jointpos();
  msg.dof = arm_joint_names_.size();
  msg.follow = true; // High-following mode
  msg.expand = 0.0;
  
  // Assuming info_.joints contains arm joints in the order corresponding to robot's physical joints
  // We need to iterate over arm_joint_names_ (which preserves discovery order)
  // But we need the values from hw_commands_position_.
  // The mapping is: arm_joint_names_[k] -> joint_name_to_index_ -> value
  
  for (const auto& name : arm_joint_names_) {
      if (joint_name_to_index_.count(name)) {
          size_t idx = joint_name_to_index_[name];
          msg.joint.push_back(hw_commands_position_[idx]);
      }
  }
  
  movej_pub_->publish(msg);
  
  // 2. Publish Gripper Command
  if (has_gripper_) {
      auto gripper_msg = rm_ros_interfaces::msg::Gripperset();
      size_t idx = gripper_joint_index_;
      double cmd_pos = hw_commands_position_[idx];
      
      // Map 0.0 - 0.07 (meters) to 0 - 1000
      // Clamp to valid range
      if (cmd_pos < 0.0) cmd_pos = 0.0;
      if (cmd_pos > 0.07) cmd_pos = 0.07;
      
      uint16_t mapped_pos = static_cast<uint16_t>((cmd_pos / 0.07) * 1000.0);
      gripper_msg.position = mapped_pos;
      gripper_msg.block = false;
      
      gripper_pub_->publish(gripper_msg);
  }
  
  return hardware_interface::return_type::OK;
}

void RmHardwareInterface::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string name = msg->name[i];
        
        // Handle name mismatch: "joint1" vs "joint_1"
        // Try direct match first
        if (joint_name_to_index_.count(name)) {
            size_t idx = joint_name_to_index_[name];
            hw_states_position_[idx] = msg->position[i];
            if (msg->velocity.size() > i) hw_states_velocity_[idx] = msg->velocity[i];
        } 
        // Try adding "_" if missing (joint1 -> joint_1)
        else {
             std::string fixed_name = name;
             if (name.find("joint") == 0 && name.length() > 5 && name[5] != '_') {
                 fixed_name = "joint_" + name.substr(5);
             }
             if (joint_name_to_index_.count(fixed_name)) {
                size_t idx = joint_name_to_index_[fixed_name];
                hw_states_position_[idx] = msg->position[i];
                if (msg->velocity.size() > i) hw_states_velocity_[idx] = msg->velocity[i];
             }
        }
    }
}

void RmHardwareInterface::hand_status_callback(const rm_ros_interfaces::msg::Handstatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_gripper_) {
        // Map 0-1000 to 0.0 - 0.07
        // Use index 0
        uint16_t raw_pos = msg->hand_pos[0];
        double pos = (static_cast<double>(raw_pos) / 1000.0) * 0.07;
        hw_states_position_[gripper_joint_index_] = pos;
    }
}

}  // namespace rm_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rm_hardware_interface::RmHardwareInterface, hardware_interface::SystemInterface)
