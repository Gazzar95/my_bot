#ifndef MOBILE_ROBOT_HARDWARE__PROTOCOL_UTILS_HPP_
#define MOBILE_ROBOT_HARDWARE__PROTOCOL_UTILS_HPP_

#include <string>

namespace mobile_robot_hardware
{

struct FeedbackData
{
  double left_vel{0.0};
  double right_vel{0.0};
  double left_pos{0.0};
  double right_pos{0.0};
  bool has_position{false};
};

std::string make_cmd_packet(double left_cmd_rad_s, double right_cmd_rad_s);
bool parse_feedback_line(const std::string & line, FeedbackData & out);

}  // namespace mobile_robot_hardware

#endif  // MOBILE_ROBOT_HARDWARE__PROTOCOL_UTILS_HPP_
