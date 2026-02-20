#ifndef MOBILE_ROBOT_HARDWARE__MOBILE_ROBOT_HARDWARE_HPP_
#define MOBILE_ROBOT_HARDWARE__MOBILE_ROBOT_HARDWARE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mobile_robot_hardware
{

class MobileRobotHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MobileRobotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool open_serial();
  void close_serial();
  bool read_feedback_line(std::string & line);
  bool parse_feedback(
    const std::string & line,
    double & left_vel,
    double & right_vel,
    double & left_pos,
    double & right_pos,
    bool & has_position) const;
  int to_baud(int baud_rate) const;

  rclcpp::Logger logger_ = rclcpp::get_logger("MobileRobotHardware");

  std::string serial_port_ = "/dev/ttyUSB0";
  int baud_rate_ = 115200;
  int serial_fd_ = -1;
  std::string read_buffer_;

  bool connected_ = false;

  std::vector<double> hw_commands_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_positions_;
};

}  // namespace mobile_robot_hardware

#endif  // MOBILE_ROBOT_HARDWARE__MOBILE_ROBOT_HARDWARE_HPP_
