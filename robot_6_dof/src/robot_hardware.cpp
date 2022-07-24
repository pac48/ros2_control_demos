#include "robot_6_dof_hardware/robot_hardware.hpp"
#include <string>
#include <vector>

namespace robot_6_dof_hardware {
  CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // robot has 6 joints and 2 interfaces
    joint_position_.assign(6, 0);
    joint_velocities_.assign(6, 0);
    joint_position_command_.assign(6, 0);
    joint_velocities_command_.assign(6, 0);

    // force sensor has 6 readings
    ft_states_.assign(6, 0);
    ft_command_.assign(6, 0);

    for (const auto &joint: info_.joints) {
      for (const auto &interface: joint.state_interfaces) {
        joint_interfaces[interface.name].push_back(joint.name);
      }
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto &joint_name: joint_interfaces["position"]) {
      state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["velocity"]) {
      state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
    state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
    state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
    state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
    state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
    state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto &joint_name: joint_interfaces["position"]) {
      command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto &joint_name: joint_interfaces["velocity"]) {
      command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
    command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
    command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
    command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
    command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
    command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);

    return command_interfaces;
  }

  return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
    // TODO set sensor_states_ values from subscriber

    for (auto i = 0ul; i < joint_velocities_command_.size(); i++) {
      joint_velocities_[i] = joint_velocities_command_[i];
      joint_position_[i] += joint_velocities_command_[i]*period.seconds();
    }

    for (auto i = 0ul; i < joint_position_command_.size(); i++) {
      joint_position_[i] = joint_position_command_[i];
    }

    return return_type::OK;
  }

}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(robot_6_dof_hardware::RobotSystem, hardware_interface::SystemInterface)
