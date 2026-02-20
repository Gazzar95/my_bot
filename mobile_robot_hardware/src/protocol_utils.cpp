#include "mobile_robot_hardware/protocol_utils.hpp"

#include <sstream>
#include <string>

namespace mobile_robot_hardware
{

std::string make_cmd_packet(double left_cmd_rad_s, double right_cmd_rad_s)
{
  std::ostringstream oss;
  oss << "CMD " << left_cmd_rad_s << " " << right_cmd_rad_s << "\n";
  return oss.str();
}

bool parse_feedback_line(const std::string & line, FeedbackData & out)
{
  std::istringstream iss(line);
  std::string tag;
  iss >> tag;

  if (tag != "FB")
  {
    return false;
  }

  if (!(iss >> out.left_vel >> out.right_vel))
  {
    return false;
  }

  if (iss >> out.left_pos >> out.right_pos)
  {
    out.has_position = true;
  }
  else
  {
    out.left_pos = 0.0;
    out.right_pos = 0.0;
    out.has_position = false;
  }

  return true;
}

}  // namespace mobile_robot_hardware
