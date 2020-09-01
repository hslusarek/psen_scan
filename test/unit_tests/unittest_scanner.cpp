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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <psen_scan/degree_to_rad.h>
#include <psen_scan/scanner.h>
#include <psen_scan/scanner_configuration.h>
#include <psen_scan/scanner_controller.h>
#include <psen_scan/mock_scanner_controller.h>

using namespace psen_scan;

using ::testing::_;

namespace psen_scan_test
{
class ScannerTest : public testing::Test
{
protected:
  ScannerTest() : mock_scanner_controller_(std::make_shared<MockScannerController>())
  {
  }

  std::shared_ptr<MockScannerController> mock_scanner_controller_;
};

TEST_F(ScannerTest, testConstructorSuccess)
{
  EXPECT_NO_THROW(ScannerImpl(std::make_shared<MockScannerController>()));
}

TEST_F(ScannerTest, testStart)
{
  ScannerImpl scanner(mock_scanner_controller_);
  EXPECT_CALL(*mock_scanner_controller_, start()).Times(1);
  scanner.start();
}

TEST_F(ScannerTest, testStop)
{
  ScannerImpl scanner(mock_scanner_controller_);
  EXPECT_CALL(*mock_scanner_controller_, stop()).Times(1);
  scanner.stop();
}

TEST_F(ScannerTest, testGetCompleteScan)
{
  ScannerImpl scanner(mock_scanner_controller_);
  EXPECT_THROW(scanner.getCompleteScan(), LaserScanBuildFailure);
}

}  // namespace psen_scan_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
