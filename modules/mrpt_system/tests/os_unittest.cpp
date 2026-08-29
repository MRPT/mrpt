/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/core/config.h>  // MRPT_OS_*()
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>

namespace os = mrpt::system::os;

TEST(os, sprintfAndVsprintfWrappers)
{
  std::array<char, 64> buf{};
  EXPECT_EQ(os::sprintf(buf.data(), buf.size(), "%s-%i", "abc", 7), 5);
  EXPECT_EQ(std::string(buf.data()), "abc-7");
}

TEST(os, stringCopyAndConcat)
{
  std::array<char, 32> buf{};

  EXPECT_EQ(os::strcpy(buf.data(), buf.size(), "hello"), buf.data());
  EXPECT_EQ(std::string(buf.data()), "hello");

  EXPECT_EQ(os::strcat(buf.data(), buf.size(), ", world"), buf.data());
  EXPECT_EQ(std::string(buf.data()), "hello, world");
}

TEST(os, stringComparisons)
{
  EXPECT_EQ(os::_strcmp("abc", "abc"), 0);
  EXPECT_NE(os::_strcmp("abc", "abd"), 0);

  // Case-insensitive:
  EXPECT_EQ(os::_strcmpi("AbC", "aBc"), 0);
  EXPECT_NE(os::_strcmpi("abc", "abd"), 0);

  // Length-limited:
  EXPECT_EQ(os::_strncmp("abcXYZ", "abcQRS", 3), 0);
  EXPECT_NE(os::_strncmp("abcXYZ", "abcQRS", 4), 0);

  EXPECT_EQ(os::_strnicmp("ABCxyz", "abcQRS", 3), 0);
  EXPECT_NE(os::_strnicmp("ABCxyz", "abcQRS", 4), 0);
}

TEST(os, memcpyWrapper)
{
  const std::array<char, 5> src = {'h', 'e', 'l', 'l', 'o'};
  std::array<char, 8> dst{};

  os::memcpy(dst.data(), dst.size(), src.data(), src.size());
  EXPECT_EQ(std::string(dst.data(), src.size()), "hello");
}

TEST(os, stringToInteger)
{
  char* endptr = nullptr;
  EXPECT_EQ(os::_strtoll("-1234", &endptr, 10), -1234);
  EXPECT_EQ(os::_strtoull("1234", &endptr, 10), 1234U);

  // Non-decimal bases:
  EXPECT_EQ(os::_strtoll("ff", &endptr, 16), 255);
  EXPECT_EQ(os::_strtoull("777", &endptr, 8), 511U);
}

TEST(os, fopenAndFclose)
{
  const std::string fname = mrpt::system::getTempFileName() + "_os_fopen";

  {
    FILE* f = os::fopen(fname, "wb");
    ASSERT_NE(f, nullptr);
    EXPECT_GT(os::fprintf(f, "%s=%i\n", "x", 3), 0);
    os::fclose(f);
  }
  {
    // The `const char*` overload as well:
    FILE* f = os::fopen(fname.c_str(), "rb");
    ASSERT_NE(f, nullptr);
    os::fclose(f);
  }

  // Opening a missing file yields a null pointer, not an exception:
  EXPECT_EQ(os::fopen(mrpt::system::getTempFileName() + "_missing", "rb"), nullptr);

  // ...and closing a null handle is rejected:
  EXPECT_ANY_THROW(os::fclose(nullptr));

  std::remove(fname.c_str());
}

TEST(os, mrptLicenseTextIsAvailable)
{
  const std::string& lic = mrpt::system::getMRPTLicense();
  EXPECT_FALSE(lic.empty());
  EXPECT_NE(lic.find("Mobile Robot Programming Toolkit"), std::string::npos);
  EXPECT_NE(lic.find("BSD License"), std::string::npos);

  // It is memoized, so a second call returns the very same string:
  EXPECT_EQ(&lic, &mrpt::system::getMRPTLicense());
}

TEST(os, consoleColorAndStyleIsANoOpOutsideATerminal)
{
  using namespace mrpt::system;

  // Under a test runner, stdout/stderr are not terminals, so no escape
  // sequence is emitted; the calls must simply be harmless.
  EXPECT_NO_THROW(consoleColorAndStyle(ConsoleForegroundColor::RED));
  EXPECT_NO_THROW(consoleColorAndStyle(
      ConsoleForegroundColor::GREEN, ConsoleBackgroundColor::BLUE, ConsoleTextStyle::BOLD, true));
  EXPECT_NO_THROW(consoleColorAndStyle(ConsoleForegroundColor::DEFAULT));
}

TEST(os, findMRPTSharedDir)
{
  // Whatever it finds, the answer is memoized and stable:
  const std::string a = mrpt::system::find_mrpt_shared_dir();
  EXPECT_EQ(a, mrpt::system::find_mrpt_shared_dir());
}

#if defined(MRPT_OS_LINUX) || defined(MRPT_OS_APPLE)

TEST(os, executeCommandCapturesOutputAndExitCode)
{
  std::string output;
  EXPECT_EQ(mrpt::system::executeCommand("echo mrpt-test-output", &output), 0);
  EXPECT_NE(output.find("mrpt-test-output"), std::string::npos);

  // A non-zero exit code is reported back:
  EXPECT_NE(mrpt::system::executeCommand("exit 3"), 0);
}

TEST(os, executeCommandInAWorkingDirectory)
{
  const std::string dir = mrpt::system::getTempFileName() + "_execdir";
  ASSERT_TRUE(mrpt::system::createDirectory(dir));

  std::string output;
  EXPECT_EQ(mrpt::system::executeCommand("pwd", &output, "r", dir), 0);
  EXPECT_NE(output.find(mrpt::system::extractFileName(dir)), std::string::npos);

  mrpt::system::deleteFile(dir);
}

TEST(os, launchProcess)
{
  EXPECT_TRUE(mrpt::system::launchProcess("true"));
  EXPECT_FALSE(mrpt::system::launchProcess("false"));
}

#endif

TEST(os, loadingAMissingPluginModuleFails)
{
  std::string err;
  EXPECT_FALSE(mrpt::system::loadPluginModule("/tmp/mrpt-no-such-plugin.so", err));
  EXPECT_FALSE(err.empty());

  EXPECT_FALSE(mrpt::system::unloadPluginModule("/tmp/mrpt-no-such-plugin.so"));
}

TEST(os, loadingAnEmptyPluginListSucceeds)
{
  std::string err;
  EXPECT_TRUE(mrpt::system::loadPluginModules("", err));
  EXPECT_TRUE(mrpt::system::unloadPluginModules(""));
}
