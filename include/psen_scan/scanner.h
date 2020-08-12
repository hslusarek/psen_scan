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

#ifndef PSEN_SCAN_SCANNER_H
#define PSEN_SCAN_SCANNER_H

#include <memory>

#include "psen_scan/laserscan.h"
#include "psen_scan/scanner_configuration.h"
#include "psen_scan/scanner_controller.h"

namespace psen_scan
{
// LCOV_EXCL_START
class vScanner
{
public:
  vScanner(const ScannerConfiguration& scanner_configuration);
  virtual ~vScanner() = default;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual LaserScan getCompleteScan() = 0;

private:
  ScannerConfiguration scanner_configuration_;
};

inline vScanner::vScanner(const ScannerConfiguration& scanner_configuration)
  : scanner_configuration_(scanner_configuration)
{
}
// LCOV_EXCL_STOP

class Scanner : public vScanner
{
public:
  Scanner(const ScannerConfiguration& scanner_configuration);
  void start();
  void stop();
  LaserScan getCompleteScan();

private:
  std::unique_ptr<ScannerController> scanner_controller_;
};

}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_H
