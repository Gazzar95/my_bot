#include <gtest/gtest.h>

#include <string>

#include "mobile_robot_hardware/protocol_utils.hpp"

TEST(ProtocolUtilsTest, MakeCmdPacketUsesExpectedLayout)
{
  const std::string packet = mobile_robot_hardware::make_cmd_packet(1.25, -2.5);
  EXPECT_EQ(packet, "CMD 1.25 -2.5\n");
}

TEST(ProtocolUtilsTest, ParseFeedbackWithVelocityOnly)
{
  mobile_robot_hardware::FeedbackData data;
  ASSERT_TRUE(mobile_robot_hardware::parse_feedback_line("FB 0.4 -0.3", data));

  EXPECT_DOUBLE_EQ(data.left_vel, 0.4);
  EXPECT_DOUBLE_EQ(data.right_vel, -0.3);
  EXPECT_FALSE(data.has_position);
  EXPECT_DOUBLE_EQ(data.left_pos, 0.0);
  EXPECT_DOUBLE_EQ(data.right_pos, 0.0);
}

TEST(ProtocolUtilsTest, ParseFeedbackWithVelocityAndPosition)
{
  mobile_robot_hardware::FeedbackData data;
  ASSERT_TRUE(mobile_robot_hardware::parse_feedback_line("FB 1.0 2.0 3.0 4.0", data));

  EXPECT_DOUBLE_EQ(data.left_vel, 1.0);
  EXPECT_DOUBLE_EQ(data.right_vel, 2.0);
  EXPECT_TRUE(data.has_position);
  EXPECT_DOUBLE_EQ(data.left_pos, 3.0);
  EXPECT_DOUBLE_EQ(data.right_pos, 4.0);
}

TEST(ProtocolUtilsTest, ParseRejectsWrongPrefix)
{
  mobile_robot_hardware::FeedbackData data;
  EXPECT_FALSE(mobile_robot_hardware::parse_feedback_line("OK 1.0 2.0", data));
}

TEST(ProtocolUtilsTest, ParseRejectsMalformedNumericPayload)
{
  mobile_robot_hardware::FeedbackData data;
  EXPECT_FALSE(mobile_robot_hardware::parse_feedback_line("FB 1.0 not_a_number", data));
}
