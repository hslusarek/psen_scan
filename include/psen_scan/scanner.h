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

#ifndef PSEN_SCAN_SCANNER_H
#define PSEN_SCAN_SCANNER_H

#include <functional>
#include <memory>
#include <stdexcept>

#include "psen_scan/controller_state_machine.h"
#include "psen_scan/laserscan.h"
#include "psen_scan/msg_decoder.h"
#include "psen_scan/scanner_configuration.h"
#include "psen_scan/scanner_controller.h"
#include "psen_scan/udp_client.h"

namespace psen_scan
{
class LaserScanBuildFailure : public std::runtime_error
{
public:
  LaserScanBuildFailure(const std::string& msg = "Error while building laser scan");
};

// LCOV_EXCL_START
class Scanner
{
public:
  virtual ~Scanner() = default;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual LaserScan getCompleteScan() = 0;
};
// LCOV_EXCL_STOP

class ScannerImpl : public Scanner
{
public:
  ScannerImpl(){};
  ScannerImpl(std::shared_ptr<ScannerController> scanner_controller); //TODO Remove later
  void start();
  void stop();
  LaserScan getCompleteScan();

private:
  std::shared_ptr<ScannerController> scanner_controller_;
};

inline LaserScanBuildFailure::LaserScanBuildFailure(const std::string& msg) : std::runtime_error(msg)
{
}

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_H
