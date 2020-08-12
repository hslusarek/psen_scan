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

#include "psen_scan/scanner.h"

#include "psen_scan/scanner_controller.h"

namespace psen_scan
{
Scanner::Scanner(const ScannerConfiguration& scanner_configuration) : scanner_controller_(scanner_configuration)
{
}

void Scanner::start()
{
  scanner_controller_.start();
}

void Scanner::stop()
{
  scanner_controller_.stop();
}

LaserScan Scanner::getCompleteScan()
{
  // TODO: Add implementation in following stories
  return LaserScan(PSENscanInternalAngle(0), PSENscanInternalAngle(0), PSENscanInternalAngle(0));
}

}  // namespace psen_scan
