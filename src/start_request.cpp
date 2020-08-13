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
#include <boost/crc.hpp>
#include <endian.h>

#include <algorithm>
#include <sstream>
#include <string>

#include <psen_scan/start_request.h>

namespace psen_scan
{
StartRequest::StartRequest(const ScannerConfiguration& scanner_configuration, const uint32_t& seq_number)
  : seq_number_(seq_number), host_udp_port_(scanner_configuration.hostUDPPortRead()) // Write is deduced by the scanner
{
  setHostIP(scanner_configuration.hostIp());
}

void StartRequest::setHostIP(const uint32_t& host_ip)
{
  host_ip_ = host_ip;
}

uint32_t StartRequest::getCRC() const
{
  boost::crc_32_type result;

  result.process_bytes((char*)&seq_number_, sizeof(uint32_t));
  result.process_bytes((char*)&RESERVED_, sizeof(uint64_t));
  result.process_bytes((char*)&OPCODE_, sizeof(uint32_t));

  in_addr_t host_ip_big_endian = htobe32(host_ip_);
  result.process_bytes((char*)&host_ip_big_endian, sizeof(uint32_t));

  result.process_bytes((char*)&host_udp_port_, sizeof(uint16_t));
  result.process_bytes((char*)&device_enabled_, sizeof(uint8_t));
  result.process_bytes((char*)&intensity_enabled_, sizeof(uint8_t));
  result.process_bytes((char*)&point_in_safety_enabled_, sizeof(uint8_t));
  result.process_bytes((char*)&active_zone_set_enabled_, sizeof(uint8_t));
  result.process_bytes((char*)&io_pin_enabled_, sizeof(uint8_t));
  result.process_bytes((char*)&scan_counter_enabled_, sizeof(uint8_t));
  result.process_bytes((char*)&speed_encoder_enabled_, sizeof(uint8_t));
  result.process_bytes((char*)&diagnostics_enabled_, sizeof(uint8_t));

  result.process_bytes((char*)&master_.start_angle_, sizeof(uint16_t));
  result.process_bytes((char*)&master_.end_angle_, sizeof(uint16_t));
  result.process_bytes((char*)&master_.resolution_, sizeof(uint16_t));

  for (const auto& slave : slaves_)
  {
    result.process_bytes((char*)&(slave.start_angle_), sizeof(uint16_t));
    result.process_bytes((char*)&(slave.end_angle_), sizeof(uint16_t));
    result.process_bytes((char*)&(slave.resolution_), sizeof(uint16_t));
  }

  return result.checksum();
}

StartRequest::RawType StartRequest::toCharArray()
{
  std::ostringstream os;

  uint32_t crc{ getCRC() };
  os.write((char*)&crc, sizeof(uint32_t));
  os.write((char*)&seq_number_, sizeof(uint32_t));
  os.write((char*)&RESERVED_, sizeof(uint64_t));
  os.write((char*)&OPCODE_, sizeof(uint32_t));

  in_addr_t host_ip_big_endian = htobe32(host_ip_);
  os.write((char*)&host_ip_big_endian, sizeof(uint32_t));

  os.write((char*)&host_udp_port_, sizeof(uint16_t));
  os.write((char*)&device_enabled_, sizeof(uint8_t));
  os.write((char*)&intensity_enabled_, sizeof(uint8_t));
  os.write((char*)&point_in_safety_enabled_, sizeof(uint8_t));
  os.write((char*)&active_zone_set_enabled_, sizeof(uint8_t));
  os.write((char*)&io_pin_enabled_, sizeof(uint8_t));
  os.write((char*)&scan_counter_enabled_, sizeof(uint8_t));
  os.write((char*)&speed_encoder_enabled_, sizeof(uint8_t));
  os.write((char*)&diagnostics_enabled_, sizeof(uint8_t));

  os.write((char*)&master_.start_angle_, sizeof(uint16_t));
  os.write((char*)&master_.end_angle_, sizeof(uint16_t));
  os.write((char*)&master_.resolution_, sizeof(uint16_t));

  for (const auto& slave : slaves_)
  {
    os.write((char*)&(slave.start_angle_), sizeof(uint16_t));
    os.write((char*)&(slave.end_angle_), sizeof(uint16_t));
    os.write((char*)&(slave.resolution_), sizeof(uint16_t));
  }

  StartRequest::RawType ret_val{};

  // TODO check limits
  std::string data_str(os.str());
  // TODO check if lengths match
  std::copy(data_str.begin(), data_str.end(), ret_val.begin());

  return ret_val;
}

}  // namespace psen_scan
