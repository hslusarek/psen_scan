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

#ifndef PSEN_SCAN_ROS_SCANNER_NODE_H
#define PSEN_SCAN_ROS_SCANNER_NODE_H

#include <string>
#include <atomic>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "psen_scan/scanner.h"

namespace psen_scan
{
/**
 * @brief Class implements a ROS-Node for the PSENscan safety laser scanner
 *
 */
class ROSScannerNode
{
public:
  ROSScannerNode(ros::NodeHandle& nh,
                 const std::string& topic,
                 const std::string& frame_id,
                 const double& x_axis_rotation,
                 std::unique_ptr<Scanner> scanner);
  sensor_msgs::LaserScan buildRosMessage(const LaserScan& laserscan);
  void processingLoop();
  void terminateProcessingLoop();

private:
  ros::NodeHandle nh_;               /**< ROS Node handler*/
  ros::Publisher pub_;               /**< ROS message publisher*/
  std::string frame_id_;             /**< Defines the name of the frame_id. Default is scanner.*/
  std::unique_ptr<Scanner> scanner_; /**< Points to an instance of the Scanner class.*/
  double x_axis_rotation_;           /**< X-axis rotation.*/
  std::atomic_bool terminate_{ false };
};
}  // namespace psen_scan

#endif  // PSEN_SCAN_ROS_SCANNER_NODE_H
