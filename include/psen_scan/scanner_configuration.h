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
#ifndef PSEN_SCAN_SCANNER_CONFIGURATION_H
#define PSEN_SCAN_SCANNER_CONFIGURATION_H

#include <array>
#include <tuple>

#include <psen_scan/psen_scan_internal_angle.h>
#include <psen_scan/scanner_data.h>

namespace psen_scan
{

typedef std::tuple<PSENscanInternalAngle, PSENscanInternalAngle, PSENscanInternalAngle> Range;

PSENscanInternalAngle const MASTER_RESOLUTION(1);
PSENscanInternalAngle const SLAVE_RESOLUTION(10);

class ScannerConfiguration
{
public:
  ScannerConfiguration(const std::string& target_ip, const uint16_t& target_udp_port);

public:
  int unsigned constexpr static NUMBER_OF_SLAVES {3};

public:
  std::string targetIp() const;

  uint16_t targetUDPPort() const;

  Range getMasterRange() const;

  std::array<Range, ScannerConfiguration::NUMBER_OF_SLAVES> getSlaveRanges() const;

  bool getSlaveEnabled(const unsigned int& index) const;

  bool intensityEnabled() const;

  bool pointInSafetyEnabled() const;

  bool activeZoneSetEnabled() const;

  bool ioPinEnabled() const;

  bool scanCounterEnabled() const;

  bool speedEncoderEnabled() const;

  bool diagnosticsEnabled() const;


private:
  class MasterDeviceConfiguration
  {
  public:
    MasterDeviceConfiguration()
    : start_angle_(MIN_SCAN_ANGLE)
    , end_angle_(MAX_SCAN_ANGLE)
    , resolution_(MASTER_RESOLUTION)
    {
    }

    Range getRange() const
    {
      return std::make_tuple(start_angle_, end_angle_, resolution_);
    }

  protected:
    MasterDeviceConfiguration(const PSENscanInternalAngle& resolution_)
    : start_angle_(MIN_SCAN_ANGLE)
    , end_angle_(MAX_SCAN_ANGLE)
    , resolution_(resolution_)
    {
    }

  private:
    /**< The following 'angle' and 'resolution' fields are all in tenth of degrees */
    PSENscanInternalAngle start_angle_;
    PSENscanInternalAngle end_angle_;
    PSENscanInternalAngle resolution_;

  };

  class SlaveDeviceConfiguration : public MasterDeviceConfiguration
  {
  public:
    SlaveDeviceConfiguration()
    : MasterDeviceConfiguration(SLAVE_RESOLUTION)
    , enabled_(false)
    {
    }

    bool enabled() const
    {
      return enabled_;
    }

  private:
    bool enabled_;
  };


private:
  std::string target_ip_;
  uint16_t target_udp_port_;

  MasterDeviceConfiguration master_;
  std::array<SlaveDeviceConfiguration, NUMBER_OF_SLAVES> slaves_;

  bool intensity_enabled_;
  bool point_in_safety_enabled_;
  bool active_zone_set_enabled_;
  bool io_pin_enabled_;
  bool scan_counter_enabled_;
  bool speed_encoder_enabled_;
  bool diagnostics_enabled_;

};

}

#endif // PSEN_SCAN_SCANNER_CONFIGURATION_H
