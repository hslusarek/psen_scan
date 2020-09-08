// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <boost/endian/conversion.hpp>

#include "psen_scan/ros_parameter_handler.h"
#include "psen_scan/default_parameters.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/degree_to_rad.h"
#include "psen_scan/get_ros_parameter_exception.h"

using namespace psen_scan;

#define DELETE_ROS_PARAM(param_name)                                                                                   \
  if (ros::param::has(param_name))                                                                                     \
  {                                                                                                                    \
    ros::param::del(param_name);                                                                                       \
  }

#define DELETE_ALL_ROS_PARAMS()                                                                                        \
  DELETE_ROS_PARAM("sensor_ip");                                                                                       \
  DELETE_ROS_PARAM("host_ip");                                                                                         \
  DELETE_ROS_PARAM("host_udp_port_data");                                                                              \
  DELETE_ROS_PARAM("host_udp_port_control");                                                                           \
  DELETE_ROS_PARAM("angle_start");                                                                                     \
  DELETE_ROS_PARAM("angle_end");                                                                                       \
  DELETE_ROS_PARAM("frame_id");                                                                                        \
  DELETE_ROS_PARAM("x_axis_rotation");

namespace psen_scan_test
{
class ROSParameterHandlerTest : public ::testing::Test
{
protected:
  ROSParameterHandlerTest()
  {
    DELETE_ALL_ROS_PARAMS();
  }

  ros::NodeHandle node_handle_;

  // Default values to set
  std::string host_ip_{ "1.2.3.5" };
  int host_udp_port_data_{ 12345 };
  int host_udp_port_control_{ 12346 };
  std::string sensor_ip_{ "1.2.3.4" };

  // Default expected values
  uint32_t expected_host_ip_{ boost::endian::native_to_big(inet_network(host_ip_.c_str())) };
  uint32_t expected_host_udp_port_data_{ boost::endian::native_to_little(static_cast<uint32_t>(host_udp_port_data_)) };
  uint32_t expected_host_udp_port_control_{ boost::endian::native_to_little(
      static_cast<uint32_t>(host_udp_port_control_)) };
  std::string expected_frame_id_{ DEFAULT_FRAME_ID };
  double expected_angle_start_degree_{ 70. };
  double expected_angle_end_degree_{ 100. };
  double expected_x_axis_rotation_degree_{ 50 };
};

class ROSRequiredParameterTest : public ROSParameterHandlerTest
{
protected:
  ROSRequiredParameterTest()
  {
    ros::param::set("sensor_ip", sensor_ip_);
    ros::param::set("host_ip", host_ip_);
    ros::param::set("host_udp_port_data", host_udp_port_data_);
    ros::param::set("host_udp_port_control", host_udp_port_control_);
  }
};

class ROSInvalidParameterTest : public ROSParameterHandlerTest
{
protected:
  ROSInvalidParameterTest()
  {
    ros::param::set("host_ip", host_ip_);
    ros::param::set("host_udp_port_data", host_udp_port_data_);
    ros::param::set("host_udp_port_control", host_udp_port_control_);
    ros::param::set("sensor_ip", sensor_ip_);
    ros::param::set("frame_id", DEFAULT_FRAME_ID);

    ros::param::set("angle_start", expected_angle_start_degree_);
    ros::param::set("angle_end", expected_angle_end_degree_);
    ros::param::set("x_axis_rotation", expected_x_axis_rotation_degree_);
  }
};

TEST_F(ROSParameterHandlerTest, test_no_param)
{
  EXPECT_THROW(RosParameterHandler param_handler(node_handle_), ParamMissingOnServer);
}

TEST_F(ROSRequiredParameterTest, test_required_params_only)
{
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_); EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
                  EXPECT_EQ(param_handler.getHostIP(), host_ip_);
                  EXPECT_EQ(param_handler.getHostUDPPortData(), expected_host_udp_port_data_);
                  EXPECT_EQ(param_handler.getHostUDPPortControl(), expected_host_udp_port_control_);
                  EXPECT_EQ(param_handler.getFrameID(), expected_frame_id_);
                  EXPECT_EQ(param_handler.getAngleStart(), DEFAULT_ANGLE_START);
                  EXPECT_EQ(param_handler.getAngleEnd(), DEFAULT_ANGLE_END);
                  EXPECT_EQ(param_handler.getXAxisRotation(), DEFAULT_X_AXIS_ROTATION););
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_sensor_ip)
{
  DELETE_ROS_PARAM("sensor_ip");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), ParamMissingOnServer);
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_host_ip)
{
  DELETE_ROS_PARAM("host_ip");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), ParamMissingOnServer);
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_host_udp_port_data)
{
  DELETE_ROS_PARAM("host_udp_port_data");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), ParamMissingOnServer);
}

TEST_F(ROSRequiredParameterTest, test_single_required_params_missing_host_udp_port_control)
{
  DELETE_ROS_PARAM("host_udp_port_control");

  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), ParamMissingOnServer);
}

TEST_F(ROSRequiredParameterTest, test_all_params)
{
  std::string frame_id = "abcdefg";
  const double expected_angle_start_degree{ 70. };
  const double expected_angle_end_degree{ 240. };
  const double expected_x_axis_rotation_degree{ 100. };
  ros::param::set("angle_start", expected_angle_start_degree);
  ros::param::set("angle_end", expected_angle_end_degree);
  ros::param::set("x_axis_rotation", expected_x_axis_rotation_degree);
  ros::param::set("frame_id", frame_id);

  RosParameterHandler param_handler(node_handle_);
  EXPECT_EQ(param_handler.getSensorIP(), sensor_ip_);
  EXPECT_EQ(param_handler.getHostIP(), host_ip_);
  EXPECT_EQ(param_handler.getHostUDPPortData(), expected_host_udp_port_data_);
  EXPECT_EQ(param_handler.getHostUDPPortControl(), expected_host_udp_port_control_);
  EXPECT_EQ(param_handler.getFrameID(), frame_id);
  EXPECT_DOUBLE_EQ(param_handler.getAngleStart(), degreeToRad(expected_angle_start_degree));
  EXPECT_DOUBLE_EQ(param_handler.getAngleEnd(), degreeToRad(expected_angle_end_degree));
  EXPECT_DOUBLE_EQ(param_handler.getXAxisRotation(), degreeToRad(expected_x_axis_rotation_degree));
}

TEST_F(ROSInvalidParameterTest, test_invalid_host_param_host_udp_port_data)
{
  ros::param::set("host_udp_port_data", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  ros::param::set("host_udp_port_data", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // Wrong Datatype, but can be converted to float
  ros::param::set("host_udp_port_data", "2.4");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // valid test
  ros::param::set("host_udp_port_data", host_udp_port_data_);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_host_param_host_udp_port_control)
{
  ros::param::set("host_udp_port_control", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  ros::param::set("host_udp_port_control", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // Wrong Datatype, but can be converted to float
  ros::param::set("host_udp_port_control", "2.4");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // valid test
  ros::param::set("host_udp_port_control", host_udp_port_control_);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_frame_id)
{
  // Set frame_id with wrong datatype (expected string) as example for wrong datatypes on expected strings
  ros::param::set("frame_id", 12);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  ros::param::set("frame_id", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // Set frame_id back to valid data type, which could be interpreted as int
  // Numbers are allowed for frame id TODO: discussion
  ros::param::set("frame_id", "125");
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_angle_start)
{
  // Set angle_start with wrong datatype (expected double) as example for wrong datatypes on expected double
  ros::param::set("angle_start", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  ros::param::set("angle_start", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // Wrong Datatype, but can be interpreted as int
  ros::param::set("angle_start", "0.21");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // valid test
  ros::param::set("angle_start", 0.35);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_angle_end)
{
  // Set angle_end with wrong datatype (expected double) as example for wrong datatypes on expected double
  ros::param::set("angle_end", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  ros::param::set("angle_end", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // Wrong Datatype, but can be converted to int
  ros::param::set("angle_end", "4.36");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // valid test
  ros::param::set("angle_end", 1.57);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

TEST_F(ROSInvalidParameterTest, test_invalid_params_x_axis_rotation)
{
  // Set x_axis_rotation with wrong datatype (expected double) as example for wrong datatypes on expected double
  ros::param::set("x_axis_rotation", "string");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  ros::param::set("x_axis_rotation", true);
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // Wrong Datatype, but can be converted to float
  ros::param::set("x_axis_rotation", "2.4");
  ASSERT_THROW(RosParameterHandler param_handler(node_handle_), WrongParameterType);

  // valid test
  ros::param::set("x_axis_rotation", 1.57);
  ASSERT_NO_THROW(RosParameterHandler param_handler(node_handle_));
}

}  // namespace psen_scan_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "integrationtest_ros_parameter_handler");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
