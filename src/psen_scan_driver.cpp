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

#include <functional>

#include "psen_scan/function_pointers.h"
#include "psen_scan/scanner.h"
#include "psen_scan/scanner_controller.h"
#include "psen_scan/ros_parameter_handler.h"
#include "psen_scan/ros_scanner_node.h"
#include "psen_scan/psen_scan_fatal_exception.h"
#include "psen_scan/default_parameters.h"
#include "psen_scan/scanner_configuration.h"
#include <rosconsole_bridge/bridge.h>
REGISTER_ROSCONSOLE_BRIDGE;

using namespace psen_scan;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psen_scan_node");
  ros::NodeHandle pnh("~");

  try
  {
    psen_scan::RosParameterHandler param_handler(pnh);

    ScannerConfiguration scanner_configuration(param_handler.getHostIP(),
                                               param_handler.getHostUDPPortData(),
                                               param_handler.getHostUDPPortControl(),
                                               param_handler.getSensorIP(),
                                               param_handler.getAngleStart(),
                                               param_handler.getAngleEnd());

    SendStartRequestCallback send_start_request_callback;
    ErrorCallback error_callback;

    std::shared_ptr<ControllerStateMachineImpl> state_machine{ new ControllerStateMachineImpl(
        send_start_request_callback) };

    MsgDecoder control_msg_decoder(std::bind(&ControllerStateMachine::processStartReplyReceivedEvent, state_machine),
                                   error_callback);
    std::shared_ptr<UdpClientImpl> control_udp_client{ new UdpClientImpl(
        std::bind(&MsgDecoder::decodeAndDispatch, &control_msg_decoder, std::placeholders::_1, std::placeholders::_2),
        error_callback,
        scanner_configuration.hostUDPPortControl(),
        scanner_configuration.clientIp(),
        CONTROL_PORT_OF_SCANNER_DEVICE) };
    MsgDecoder data_msg_decoder(std::bind(&ControllerStateMachine::processStartReplyReceivedEvent, state_machine),
                                error_callback);
    std::shared_ptr<UdpClientImpl> data_udp_client{ new UdpClientImpl(
        std::bind(&MsgDecoder::decodeAndDispatch, &data_msg_decoder, std::placeholders::_1, std::placeholders::_2),
        error_callback,
        scanner_configuration.hostUDPPortData(),
        scanner_configuration.clientIp(),
        DATA_PORT_OF_SCANNER_DEVICE) };

    ScannerControllerImpl scanner_controller(scanner_configuration, state_machine, control_udp_client, data_udp_client);

    // binding callbacks
    send_start_request_callback = std::bind(&ScannerControllerImpl::sendStartRequest, &scanner_controller);
    error_callback = std::bind(&ScannerControllerImpl::handleError, &scanner_controller, std::placeholders::_1);

    std::unique_ptr<ScannerImpl> scanner{ new ScannerImpl(
        std::make_shared<ScannerControllerImpl>(scanner_controller)) };

    ROSScannerNode ros_scanner_node(
        pnh, DEFAULT_PUBLISH_TOPIC, param_handler.getFrameID(), param_handler.getXAxisRotation());
    ros_scanner_node.processingLoop();
  }
  catch (PSENScanFatalException& e)
  {
    std::cerr << e.what() << std::endl;
  }
  catch (std::exception& e)
  {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
