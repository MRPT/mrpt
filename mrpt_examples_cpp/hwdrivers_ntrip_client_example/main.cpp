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

#include <mrpt/core/exceptions.h>
#include <mrpt/core/get_env.h>
#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/system/os.h>

#include <chrono>
#include <cstdio>
#include <iostream>
#include <thread>

using namespace std;
using namespace mrpt;
using namespace mrpt::hwdrivers;
using namespace mrpt::system;

std::string ntrip_user = mrpt::get_env<std::string>("NTRIP_USER");
std::string ntrip_pass = mrpt::get_env<std::string>("NTRIP_PASS");

// ------------------------------------------------------
//				TestNTRIP
// ------------------------------------------------------
void TestNTRIP()
{
  const string server = "www.euref-ip.net";
  const int server_port = 2101;

  CNTRIPClient::TListMountPoints lst;
  string errMsg;

  bool ret = CNTRIPClient::retrieveListOfMountpoints(lst, errMsg, server, server_port);

  if (!ret)
  {
    std::cout << "Error: " << errMsg << "\n";
    return;
  }

  if (lst.empty())
  {
    std::cout << "Zero streams listed in caster...?"
              << "\n";
    return;
  }

  // List:
  // -----------------------------------
  for (CNTRIPClient::TListMountPoints::const_iterator it = lst.begin(); it != lst.end(); it++)
  {
    const CNTRIPClient::TMountPoint& m = *it;
    std::cout << "MOUNT: " << m.mountpoint_name << "  | COUNTRY: " << m.country_code
              << "  | NMEA?: " << m.needs_nmea << "  | FORMAT: " << m.format << " (" << m.id << ") "
              << m.extra_info << "\n";
  }

  // Now connect to a random server:
  // -----------------------------------
  CNTRIPClient ntrip;
  CNTRIPClient::NTRIPArgs params;

  CNTRIPClient::TListMountPoints::iterator it = lst.begin();
  // std::advance(it,8);

  std::cout << "Connecting to: " << it->mountpoint_name << " - " << it->id << "\n";
  params.mountpoint = it->mountpoint_name;
  params.server = server;
  params.port = server_port;

  std::cout << "Using user: " << ntrip_user << "\n";
  std::cout << "Using pass: " << ntrip_pass << "\n";
  std::cout << "(You can change them with env variables NTRIP_USER and "
               "NTRIP_PASS"
            << "\n";

  params.user = ntrip_user;
  params.password = ntrip_pass;

  string msgerr;

  if (!ntrip.open(params, msgerr))
  {
    std::cout << "ERROR: " << msgerr << "\n";
  }
  else
  {
    std::cout << "Reading stream... press any key to finish."
              << "\n";

    std::vector<uint8_t> dat;
    while (!mrpt::system::os::kbhit())
    {
      ntrip.stream_data.readAndClear(dat);
      std::cout << "Read " << dat.size() << " bytes."
                << "\n";
      std::this_thread::sleep_for(1000ms);
    }
  }
}

void TestNTRIP2()
{
  // const string server = "www.euref-ip.net";
  const string server = "193.144.251.13";
  const int server_port = 2101;

  // Now connect to a random server:
  // -----------------------------------
  CNTRIPClient ntrip;
  CNTRIPClient::NTRIPArgs params;

  params.mountpoint = "ACOR0";
  params.server = server;
  params.port = server_port;
  params.user = "";
  params.password = "";

  string msgerr;

  if (!ntrip.open(params, msgerr))
  {
    std::cout << "ERROR: " << msgerr << "\n";
  }
  else
  {
    std::cout << "Reading stream... press any key to finish."
              << "\n";

    std::vector<uint8_t> dat;
    while (!mrpt::system::os::kbhit())
    {
      ntrip.stream_data.readAndClear(dat);
      std::cout << "Read " << dat.size() << " bytes."
                << "\n";
      std::this_thread::sleep_for(1000ms);
    }
  }
}

int main()
{
  try
  {
    TestNTRIP();
    // TestNTRIP2();

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << "MRPT error: " << mrpt::exception_to_str(e) << "\n";
    return -1;
  }
}
