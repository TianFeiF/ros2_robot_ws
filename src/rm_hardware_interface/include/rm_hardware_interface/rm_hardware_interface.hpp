#ifndef RM_HARDWARE_INTERFACE_HPP_
#define RM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "rm_ros_interfaces/msg/gripperset.hpp"
#include "rm_ros_interfaces/msg/handstatus.hpp"

namespace rm_hardware_interface
{
class RmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RmHardwareInterface)

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
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void hand_status_callback(const rm_ros_interfaces::msg::Handstatus::SharedPtr msg);

  // Communication
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<rm_ros_interfaces::msg::Handstatus>::SharedPtr hand_status_sub_;
  
  rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr movej_pub_;
  rclcpp::Publisher<rm_ros_interfaces::msg::Gripperset>::SharedPtr gripper_pub_;

  // Data storage
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;
  std::vector<double> hw_commands_position_;
  std::vector<double> hw_commands_velocity_;
  
  // Mappings
  std::map<std::string, size_t> joint_name_to_index_;
  std::string gripper_joint_name_ = "";
  size_t gripper_joint_index_ = 0;
  bool has_gripper_ = false;
  
  // RealMan specific
  std::vector<std::string> arm_joint_names_; // Names expected by RealMan (or just use order)
  
  std::mutex mutex_;
  bool first_read_pass_ = true;
};

}  // namespace rm_hardware_interface

#endif  // RM_HARDWARE_INTERFACE_HPP_
