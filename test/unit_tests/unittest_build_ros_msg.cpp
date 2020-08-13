// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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

#include <string>
#include <memory>

#include <gtest/gtest.h>

#include "psen_scan/ros_scanner_node.h"
#include "psen_scan/build_ros_message_exception.h"
#include "psen_scan/mock_scanner.h"

using namespace psen_scan;

namespace psen_scan_test
{
TEST(BuildRosMsgFromLaserScanTests, buildROSMessage)
{
  ros::NodeHandle nh;
  std::unique_ptr<MockScanner> mock_scanner{ new MockScanner() };
  ROSScannerNode ros_scanner_node(nh, "node1_topic", "node1_frame", Degree(137.5), std::move(mock_scanner));

  LaserScan laser_scan_fake{ PSENscanInternalAngle(1), PSENscanInternalAngle(1), PSENscanInternalAngle(2) };
  laser_scan_fake.measures_.push_back(1);

  LaserScan laser_scan_error_1{ PSENscanInternalAngle(1), PSENscanInternalAngle(1), PSENscanInternalAngle(2) };
  laser_scan_error_1.measures_.push_back(1);
  laser_scan_error_1.measures_.push_back(2);

  LaserScan laser_scan_error_2{ PSENscanInternalAngle(0), PSENscanInternalAngle(1), PSENscanInternalAngle(2) };
  laser_scan_error_2.measures_.push_back(1);

  LaserScan laser_scan_error_3{ PSENscanInternalAngle(10), PSENscanInternalAngle(2), PSENscanInternalAngle(1) };
  laser_scan_error_3.measures_.push_back(1);

  ros_scanner_node.buildRosMessage(laser_scan_fake);
  EXPECT_THROW(ros_scanner_node.buildRosMessage(laser_scan_error_1), BuildROSMessageException);
  EXPECT_THROW(ros_scanner_node.buildRosMessage(laser_scan_error_2), BuildROSMessageException);
  EXPECT_THROW(ros_scanner_node.buildRosMessage(laser_scan_error_3), BuildROSMessageException);
}

}  // namespace psen_scan_test

int main(int argc, char** argv)
{
  ros::init(argc, argv, "unittest_build_ros_msg");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 2 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
