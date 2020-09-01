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

#include <iostream>

#include "psen_scan/ros_scanner_node.h"
#include "psen_scan/build_ros_message_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include <psen_scan/scanner_data.h>

namespace psen_scan
{
/**
 * @brief Construct a new ROSScannerNode::ROSScannerNode object
 *
 * @param nh node handle for the ROS node
 * @param topic name of the ROS topic
 * @param frame_id name of the frame id
 * @param scanner pointer ot an instance of the class Scanner
 */
ROSScannerNode::ROSScannerNode(ros::NodeHandle& nh,
                               const std::string& topic,
                               const std::string& frame_id,
                               const double& x_axis_rotation,
                               std::unique_ptr<Scanner> scanner)
  : nh_(nh), frame_id_(frame_id), scanner_(std::move(scanner)), x_axis_rotation_(x_axis_rotation)
{
  if (!scanner_)
  {
    throw PSENScanFatalException("Nullpointer isn't a valid argument!");
  }
  pub_ = nh_.advertise<sensor_msgs::LaserScan>(topic, 1);
}

/**
 * @brief Creates a ROS message from an LaserScan, which contains one scanning round.
 *
 * @param laserscan
 *
 * @return sensor_msgs::LaserScan, ROS message, ready to be published
 *
 * @throws BuildROSMessageException
 *
 */
sensor_msgs::LaserScan ROSScannerNode::buildRosMessage(const LaserScan& laserscan)
{
  // TODO Remove after implementing building of laserscans
  // LCOV_EXCL_START

  if (!laserscan.isNumberOfScansValid())
  {
    throw BuildROSMessageException("Calculated number of scans doesn't match actual number of scans!");
  }

  sensor_msgs::LaserScan ros_message;
  ros_message.header.stamp = ros::Time::now();
  ros_message.header.frame_id = frame_id_;
  ros_message.angle_min = laserscan.getMinScanAngle() - x_axis_rotation_;
  ros_message.angle_max = laserscan.getMaxScanAngle() - x_axis_rotation_;
  ros_message.angle_increment = laserscan.getScanResolution();
  ros_message.time_increment = SCAN_TIME / NUMBER_OF_SAMPLES_FULL_SCAN_MASTER;
  ros_message.scan_time = SCAN_TIME;
  ros_message.range_min = 0;
  ros_message.range_max = 10;
  ros_message.ranges.insert(ros_message.ranges.end(),
                            laserscan.getMeasurements().crbegin(),
                            laserscan.getMeasurements().crend());  // reverse order
  std::transform(ros_message.ranges.begin(), ros_message.ranges.end(), ros_message.ranges.begin(), [](float f) {
    return f * 0.001;
  });

  return ros_message;

  // LCOV_EXCL_STOP
}

void ROSScannerNode::terminateProcessingLoop()
{
  terminate_ = true;
}

/**
 * @brief endless loop for processing incoming UDP data from the laser scanner
 *
 */
void ROSScannerNode::processingLoop()
{
  ros::Rate r(10);
  scanner_->start();
  while (ros::ok() && !terminate_)
  {
    try
    {
      pub_.publish(buildRosMessage(scanner_->getCompleteScan()));
    }
    catch (const LaserScanBuildFailure& ex)
    {
      std::cout << ex.what() << std::endl;
    }
    r.sleep();
  }
  scanner_->stop();
}

}  // namespace psen_scan
