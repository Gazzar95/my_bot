#include "mobile_robot_hardware/mobile_robot_hardware.hpp"
#include "mobile_robot_hardware/protocol_utils.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <pluginlib/class_list_macros.hpp>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace mobile_robot_hardware
{

hardware_interface::CallbackReturn MobileRobotHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(logger_, "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(logger_, "Joint '%s' must expose exactly one velocity command interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_velocity_state = false;
    for (const auto & state_if : joint.state_interfaces)
    {
      if (state_if.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_state = true;
      }
    }

    if (!has_velocity_state)
    {
      RCLCPP_ERROR(logger_, "Joint '%s' must expose a velocity state interface", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  serial_port_ = info_.hardware_parameters.count("serial_port")
                   ? info_.hardware_parameters.at("serial_port")
                   : "/dev/ttyUSB0";

  if (info_.hardware_parameters.count("baud_rate"))
  {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }

  hw_commands_ = {0.0, 0.0};
  hw_velocities_ = {0.0, 0.0};
  hw_positions_ = {0.0, 0.0};

  RCLCPP_INFO(logger_, "Configured hardware with serial port '%s' @ %d baud", serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MobileRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MobileRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MobileRobotHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    RCLCPP_ERROR(logger_, "Failed to open serial connection");
    return hardware_interface::CallbackReturn::ERROR;
  }

  connected_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileRobotHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MobileRobotHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;
  write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.0));
  close_serial();
  connected_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MobileRobotHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!connected_)
  {
    return hardware_interface::return_type::ERROR;
  }

  std::string line;
  if (!read_feedback_line(line))
  {
    hw_positions_[0] += hw_velocities_[0] * period.seconds();
    hw_positions_[1] += hw_velocities_[1] * period.seconds();
    return hardware_interface::return_type::OK;
  }

  double left_vel = 0.0;
  double right_vel = 0.0;
  double left_pos = 0.0;
  double right_pos = 0.0;
  bool has_position = false;
  if (!parse_feedback(line, left_vel, right_vel, left_pos, right_pos, has_position))
  {
    return hardware_interface::return_type::OK;
  }

  hw_velocities_[0] = left_vel;
  hw_velocities_[1] = right_vel;

  if (has_position)
  {
    hw_positions_[0] = left_pos;
    hw_positions_[1] = right_pos;
  }
  else
  {
    hw_positions_[0] += hw_velocities_[0] * period.seconds();
    hw_positions_[1] += hw_velocities_[1] * period.seconds();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MobileRobotHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!connected_ || serial_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  // Serial command expected by firmware:
  // CMD <left_wheel_rad_s> <right_wheel_rad_s>
  const std::string packet = make_cmd_packet(hw_commands_[0], hw_commands_[1]);

  const ssize_t written = ::write(serial_fd_, packet.c_str(), packet.size());
  if (written < 0)
  {
    RCLCPP_ERROR(logger_, "Serial write failed: %s", std::strerror(errno));
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool MobileRobotHardware::open_serial()
{
  close_serial();

  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(logger_, "Could not open serial port '%s': %s", serial_port_.c_str(), std::strerror(errno));
    return false;
  }

  termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcgetattr failed: %s", std::strerror(errno));
    close_serial();
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  const speed_t speed = static_cast<speed_t>(to_baud(baud_rate_));
  if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0)
  {
    RCLCPP_ERROR(logger_, "Failed to set baud rate %d", baud_rate_);
    close_serial();
    return false;
  }

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(logger_, "tcsetattr failed: %s", std::strerror(errno));
    close_serial();
    return false;
  }

  tcflush(serial_fd_, TCIFLUSH);
  read_buffer_.clear();
  return true;
}

void MobileRobotHardware::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool MobileRobotHardware::read_feedback_line(std::string & line)
{
  if (serial_fd_ < 0)
  {
    return false;
  }

  char buf[256];
  const ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
  if (n > 0)
  {
    read_buffer_.append(buf, static_cast<size_t>(n));
  }

  const std::size_t newline_pos = read_buffer_.find('\n');
  if (newline_pos == std::string::npos)
  {
    return false;
  }

  line = read_buffer_.substr(0, newline_pos);
  read_buffer_.erase(0, newline_pos + 1);
  return true;
}

bool MobileRobotHardware::parse_feedback(
  const std::string & line,
  double & left_vel,
  double & right_vel,
  double & left_pos,
  double & right_pos,
  bool & has_position) const
{
  FeedbackData data;
  if (!parse_feedback_line(line, data))
  {
    return false;
  }

  left_vel = data.left_vel;
  right_vel = data.right_vel;
  left_pos = data.left_pos;
  right_pos = data.right_pos;
  has_position = data.has_position;

  return true;
}

int MobileRobotHardware::to_baud(int baud_rate) const
{
  switch (baud_rate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      RCLCPP_WARN(logger_, "Unsupported baud %d, defaulting to 115200", baud_rate);
      return B115200;
  }
}

}  // namespace mobile_robot_hardware

PLUGINLIB_EXPORT_CLASS(
  mobile_robot_hardware::MobileRobotHardware,
  hardware_interface::SystemInterface)
