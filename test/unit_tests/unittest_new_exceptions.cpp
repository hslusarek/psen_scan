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
#include "psen_scan/build_ros_message_exception.h"
#include "psen_scan/get_ros_parameter_exception.h"
#include "psen_scan/psen_scan_fatal_exception.h"

using namespace psen_scan;

namespace psen_scan_test
{
TEST(BuildROSMessageExceptionTest, new_build_ros_message_exception)
{
  std::string except_str = "BuildROSMessageException";
  std::unique_ptr<BuildROSMessageException> e(new BuildROSMessageException(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(GetROSParameterExceptionTest, new_get_ros_parameter_exception)
{
  std::string except_str = "GetROSParameterException";
  std::unique_ptr<ParamMissingOnServer> e(new ParamMissingOnServer(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(PSENScanFatalExceptionTest, new_psen_scan_fatal_exception)
{
  std::string except_str = "PSENScanFatalException";
  std::unique_ptr<PSENScanFatalException> e(new PSENScanFatalException(except_str));
  EXPECT_EQ(except_str, e->what());
}
}  // namespace psen_scan_test