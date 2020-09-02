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

#include <chrono>
#include <thread>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <pilz_testutils/async_test.h>

namespace psen_scan_test
{

class RosScanDriverTest : public testing::Test, public testing::AsyncTest
{
protected:
  ros::NodeHandle nh_priv_{ "~" };
};

TEST_F(RosScanDriverTest, nodeStarted)
{
  std::this_thread::sleep_for(std::chrono::seconds(600));
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_ros_scan_driver");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
