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

#include <algorithm>

#include "psen_scan/ros_parameter_handler.h"
#include "psen_scan/get_ros_parameter_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/default_parameters.h"
#include "psen_scan/scanner_data.h"

namespace psen_scan
{
/**
 * @brief Construct a new Ros Parameter Handler:: Ros Parameter Handler object
 *
 * Gets all Parameters from Parameter Server.
 *
 * @param nh Nodehandle from which parameters will be fetched
 *
 * @throws PSENScanFatalException
 */
RosParameterHandler::RosParameterHandler(const ros::NodeHandle& nh)
  : nh_(nh)
  , host_ip_()
  , host_udp_port_data_()
  , host_udp_port_control_()
  , frame_id_(DEFAULT_FRAME_ID)
  , angle_start_(DEFAULT_ANGLE_START)
  , angle_end_(DEFAULT_ANGLE_END)
  , x_axis_rotation_(DEFAULT_X_AXIS_ROTATION)
{
  updateAllParamsFromParamServer();
}

/**
 * @brief Update all Parameters from ROS Parameter Server
 *
 */
void RosParameterHandler::updateAllParamsFromParamServer()
{
  // required parameters first
  getRequiredParamFromParamServer<std::string>("host_ip", host_ip_);
  getRequiredParamFromParamServer<int>("host_udp_port_data", host_udp_port_data_);
  getRequiredParamFromParamServer<int>("host_udp_port_control", host_udp_port_control_);
  getRequiredParamFromParamServer<std::string>("sensor_ip", sensor_ip_);
  getOptionalParamFromParamServer<std::string>("frame_id", frame_id_);

  double angle_start;
  if (getOptionalParamFromParamServer<double>("angle_start", angle_start))
  {
    angle_start_ = degreeToRad(angle_start);
  }

  double angle_end;
  if (getOptionalParamFromParamServer<double>("angle_end", angle_end))
  {
    angle_end_ = degreeToRad(angle_end);
  }

  double x_axis_rotation;
  if (getOptionalParamFromParamServer<double>("x_axis_rotation", x_axis_rotation))
  {
    x_axis_rotation_ = degreeToRad(x_axis_rotation);
  }
}

/**
 * @brief Gets one required ROS-parameter from parameter server
 *
 * @tparam T Type of parameter to fetch
 * @param key Key for the parameter on parameter-server
 * @param param variable where the parameter is stored
 * @throws GetROSParameterException
 */
template <class T>
void RosParameterHandler::getRequiredParamFromParamServer(const std::string& key, T& param)
{
  if (!nh_.hasParam(key))
  {
    throw ParamMissingOnServer("Parameter " + key + " doesn't exist on parameter server.");
  }

  if (!nh_.getParam(key, param))
  {
    throw WrongParameterType("Parameter " + key + " has wrong datatype on parameter server.");
  }
  return;
}

/**
 * @brief Gets one optional ROS-parameter from parameter server
 *
 * @tparam T Type of parameter to fetch
 * @param key Key for the parameter on parameter-server
 * @param param variable where the parameter is stored
 * @return true for sucess and false if the parameter does not exist
 *
 * @throws GetROSParameterException
 */
template <class T>
bool RosParameterHandler::getOptionalParamFromParamServer(const std::string& key, T& param)
{
  if (!nh_.hasParam(key))
  {
    ROS_WARN_STREAM("Parameter " + key + " doesn't exist on parameter server. Proceeding with default configuration.");
    return false;
  }
  if (!nh_.getParam(key, param))
  {
    throw WrongParameterType("Parameter " + key + " has wrong datatype on parameter server.");
  }
  return true;
}

/**
 * @brief Getter method for host_ip_
 */
std::string RosParameterHandler::getHostIP() const
{
  return host_ip_;
}

/**
 * @brief Getter method for host_udp_port_
 *
 * @return uint32_t
 */
uint32_t RosParameterHandler::getHostUDPPortData() const
{
  return host_udp_port_data_;
}

/**
 * @brief Getter method for host_udp_port_
 *
 * @return uint32_t
 */
uint32_t RosParameterHandler::getHostUDPPortControl() const
{
  return host_udp_port_control_;
}

/**
 * @brief Getter Method for sensor_ip_
 *
 * @return std::string
 */
std::string RosParameterHandler::getSensorIP() const
{
  return sensor_ip_;
}

/**
 * @brief Getter method for frame_id_
 *
 * @return std::string
 */
std::string RosParameterHandler::getFrameID() const
{
  return frame_id_;
}

double RosParameterHandler::getAngleStart() const
{
  return angle_start_;
}

double RosParameterHandler::getAngleEnd() const
{
  return angle_end_;
}

double RosParameterHandler::getXAxisRotation() const
{
  return x_axis_rotation_;
}

}  // namespace psen_scan
