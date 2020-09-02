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

#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <pilz_testutils/async_test.h>

#include "psen_scan/degree_to_rad.h"
#include "psen_scan/ros_scanner_node.h"
#include "psen_scan/laserscan.h"
#include "psen_scan/mock_scanner_impl.h"
#include "psen_scan/psen_scan_fatal_exception.h"

using namespace psen_scan;
using namespace psen_scan_test;
using ::testing::DoAll;
using ::testing::Return;

namespace psen_scan
{
static constexpr std::chrono::seconds LOOP_END_TIMEOUT{ 3 };

static constexpr int QUEUE_SIZE{ 10 };

static const std::string LASER_SCAN_RECEIVED{ "LASER_SCAN_RECEIVED" };

static constexpr double DEFAULT_X_AXIS_ROTATION{ degreeToRad(137.5) };

class SubscriberMock
{
public:
  void initialize(ros::NodeHandle& nh);

  MOCK_METHOD1(callback, void(const sensor_msgs::LaserScan& msg));

private:
  ros::Subscriber subscriber_;
};

inline void SubscriberMock::initialize(ros::NodeHandle& nh)
{
  subscriber_ = nh.subscribe("scan", QUEUE_SIZE, &SubscriberMock::callback, this);
}

class RosScannerNodeTests : public testing::Test, public testing::AsyncTest
{
protected:
  ros::NodeHandle nh_priv_{ "~" };
};

TEST_F(RosScannerNodeTests, testScanTopicReceived)
{
  SubscriberMock subscriber;
  EXPECT_CALL(subscriber, callback(::testing::_)).WillOnce(ACTION_OPEN_BARRIER_VOID(LASER_SCAN_RECEIVED));

  LaserScan laser_scan_fake(0.02, 0.03, 0.05);
  laser_scan_fake.getMeasurements().push_back(1);
  //std::unique_ptr<MockScannerImpl> mock_scanner{ new MockScannerImpl() };

  ROSScannerNodeImpl<MockScannerImpl> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION);
  EXPECT_CALL(ros_scanner_node.scanner_, getCompleteScan()).WillRepeatedly(Return(laser_scan_fake));

  subscriber.initialize(nh_priv_);
  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.processingLoop(); });
  BARRIER(LASER_SCAN_RECEIVED);
  ros_scanner_node.terminateProcessingLoop();
  EXPECT_EQ(loop.wait_for(LOOP_END_TIMEOUT), std::future_status::ready);
}

ACTION(ThrowScanBuildFailure)
{
  throw LaserScanBuildFailure();
}

TEST_F(RosScannerNodeTests, testScanBuildFailure)
{
  SubscriberMock subscriber;
  EXPECT_CALL(subscriber, callback(::testing::_)).Times(1).WillOnce(ACTION_OPEN_BARRIER_VOID(LASER_SCAN_RECEIVED));

  LaserScan laser_scan_fake(0.02, 0.03, 0.05);
  laser_scan_fake.getMeasurements().push_back(1);

  ROSScannerNodeImpl<MockScannerImpl> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION);
  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, getCompleteScan())
        .Times(100)
        .WillRepeatedly(DoAll(ThrowScanBuildFailure(), Return(laser_scan_fake)));
    EXPECT_CALL(ros_scanner_node.scanner_, getCompleteScan()).Times(1).WillRepeatedly(Return(laser_scan_fake));
  }


  subscriber.initialize(nh_priv_);
  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.processingLoop(); });
  BARRIER(LASER_SCAN_RECEIVED);
  ros_scanner_node.terminateProcessingLoop();
  EXPECT_EQ(loop.wait_for(LOOP_END_TIMEOUT), std::future_status::ready);
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_ros_scanner_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
