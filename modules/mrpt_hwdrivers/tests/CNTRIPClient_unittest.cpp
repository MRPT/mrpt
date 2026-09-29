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
#include <mrpt/hwdrivers/CNTRIPClient.h>
#include <mrpt/system/string_utils.h>

#include <chrono>
#include <thread>

#include "fake_tcp_device.h"

using namespace mrpt::hwdrivers;
using mrpt::hwdrivers::testing::FakeTcpDevice;

namespace
{
/** A minimal NTRIP caster: it answers the HTTP-like request with a status
 * line, and then streams some "correction data". */
FakeTcpDevice::Handler casterHandler(const std::string& expectedAuthB64 = {})
{
  return [expectedAuthB64](const std::string& req) -> std::string
  {
    if (req.rfind("GET /", 0) != 0)
    {
      return {};  // not a request: e.g. data sent back by the client
    }
    if (!expectedAuthB64.empty() &&
        req.find("Authorization: Basic " + expectedAuthB64) == std::string::npos)
    {
      return "HTTP/1.0 401 Unauthorized\r\n\r\n";
    }
    if (req.find("GET /MISSING") == 0)
    {
      return "HTTP/1.0 404 Not Found\r\n\r\n";
    }
    return "ICY 200 OK\r\nRTCM-DATA-0123456789";
  };
}

/** Waits until the client has received at least `n` bytes of stream. */
std::vector<uint8_t> waitForStream(CNTRIPClient& c, size_t n)
{
  std::vector<uint8_t> all;
  for (int i = 0; i < 300 && all.size() < n; i++)
  {
    std::vector<uint8_t> chunk;
    c.stream_data.readAndClear(chunk);
    all.insert(all.end(), chunk.begin(), chunk.end());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return all;
}
}  // namespace

TEST(CNTRIPClient, InvalidArguments)
{
  CNTRIPClient c;
  std::string err;
  CNTRIPClient::NTRIPArgs args;
  args.server = "127.0.0.1";
  EXPECT_FALSE(c.open(args, err));  // empty mountpoint
  EXPECT_FALSE(err.empty());

  args.mountpoint = "MOUNT";
  args.server.clear();
  EXPECT_FALSE(c.open(args, err));
  EXPECT_FALSE(err.empty());

  c.sendBackToServer("");  // nothing to queue
}

TEST(CNTRIPClient, ReceivesStreamAndSendsDataBack)
{
  FakeTcpDevice dev(casterHandler());
  ASSERT_TRUE(dev.isReady());

  CNTRIPClient c;
  CNTRIPClient::NTRIPArgs args;
  args.server = "127.0.0.1";
  args.port = dev.port();
  args.mountpoint = "MOUNT";
  std::string err;
  ASSERT_TRUE(c.open(args, err)) << err;

  // All the data is received, including the part that came together with the
  // response header:
  const auto data = waitForStream(c, 20);
  ASSERT_EQ(data.size(), 20u);
  EXPECT_EQ(std::string(data.begin(), data.end()), "RTCM-DATA-0123456789");

  // What the client asked for:
  const auto reqs = dev.requests();
  ASSERT_GE(reqs.size(), 1u);
  EXPECT_EQ(reqs[0].rfind("GET /MOUNT HTTP/1.0", 0), 0u);
  EXPECT_NE(reqs[0].find("User-Agent: NTRIP"), std::string::npos);
  EXPECT_EQ(reqs[0].find("Authorization"), std::string::npos);

  // Data queued for the server (e.g. a GGA sentence) is sent:
  c.sendBackToServer("$GPGGA,dummy*00\r\n");
  bool sent = false;
  for (int i = 0; i < 200 && !sent; i++)
  {
    for (const auto& r : dev.requests())
    {
      sent = sent || r.find("$GPGGA") != std::string::npos;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(sent);

  c.close();
}

TEST(CNTRIPClient, HttpStyleResponseHeaders)
{
  // NTRIP v2 casters reply with full HTTP headers:
  FakeTcpDevice dev(
      [](const std::string& req) -> std::string
      {
        if (req.rfind("GET /", 0) != 0)
        {
          return {};
        }
        return "HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0\r\nContent-Type: "
               "gnss/data\r\n\r\nABCDEF";
      });
  ASSERT_TRUE(dev.isReady());
  CNTRIPClient c;
  CNTRIPClient::NTRIPArgs args;
  args.server = "127.0.0.1";
  args.port = dev.port();
  args.mountpoint = "MOUNT";
  std::string err;
  ASSERT_TRUE(c.open(args, err)) << err;
  const auto data = waitForStream(c, 6);
  EXPECT_EQ(std::string(data.begin(), data.end()), "ABCDEF");
}

TEST(CNTRIPClient, IncompleteResponseHeaderIsNotAccepted)
{
  // The status is fine but the headers never end: whatever follows must not be
  // taken as stream data.
  FakeTcpDevice dev(
      [](const std::string& req) -> std::string
      {
        if (req.rfind("GET /", 0) != 0)
        {
          return {};
        }
        return "HTTP/1.1 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n";
      });
  ASSERT_TRUE(dev.isReady());
  CNTRIPClient c;
  CNTRIPClient::NTRIPArgs args;
  args.server = "127.0.0.1";
  args.port = dev.port();
  args.mountpoint = "MOUNT";
  std::string err;
  EXPECT_FALSE(c.open(args, err));
  EXPECT_FALSE(err.empty());
}

TEST(CNTRIPClient, BasicAuthenticationIsSent)
{
  std::string b64;
  mrpt::system::encodeBase64(std::vector<uint8_t>{'u', 's', 'e', 'r', ':', 'p', 'w'}, b64);
  FakeTcpDevice dev(casterHandler(b64));
  ASSERT_TRUE(dev.isReady());

  CNTRIPClient c;
  CNTRIPClient::NTRIPArgs args;
  args.server = "127.0.0.1";
  args.port = dev.port();
  args.mountpoint = "MOUNT";
  args.user = "user";
  args.password = "pw";
  std::string err;
  EXPECT_TRUE(c.open(args, err)) << err;
}

TEST(CNTRIPClient, ServerErrorsAreReported)
{
  {
    // Wrong credentials:
    FakeTcpDevice dev(casterHandler("Zm9vOmJhcg=="));
    ASSERT_TRUE(dev.isReady());
    CNTRIPClient c;
    CNTRIPClient::NTRIPArgs args;
    args.server = "127.0.0.1";
    args.port = dev.port();
    args.mountpoint = "MOUNT";
    std::string err;
    EXPECT_FALSE(c.open(args, err));
    EXPECT_NE(err.find("Authentication"), std::string::npos) << err;
  }
  {
    // Unknown mountpoint:
    FakeTcpDevice dev(casterHandler());
    ASSERT_TRUE(dev.isReady());
    CNTRIPClient c;
    CNTRIPClient::NTRIPArgs args;
    args.server = "127.0.0.1";
    args.port = dev.port();
    args.mountpoint = "MISSING";
    std::string err;
    EXPECT_FALSE(c.open(args, err));
    EXPECT_NE(err.find("Error trying to connect"), std::string::npos) << err;
  }
  {
    // Nothing listening:
    CNTRIPClient c;
    CNTRIPClient::NTRIPArgs args;
    args.server = "127.0.0.1";
    args.port = 18799;
    args.mountpoint = "MOUNT";
    std::string err;
    EXPECT_FALSE(c.open(args, err));
    EXPECT_FALSE(err.empty());
  }
}

TEST(CNTRIPClient, CanBeOpenedAgainAfterAFailureAndAfterASuccess)
{
  CNTRIPClient c;
  CNTRIPClient::NTRIPArgs bad;
  bad.server = "127.0.0.1";
  bad.port = 18799;
  bad.mountpoint = "MOUNT";
  std::string err;
  EXPECT_FALSE(c.open(bad, err));

  // Retrying with a working server must be possible...
  FakeTcpDevice dev(casterHandler());
  ASSERT_TRUE(dev.isReady());
  CNTRIPClient::NTRIPArgs good = bad;
  good.port = dev.port();
  ASSERT_TRUE(c.open(good, err)) << err;
  EXPECT_GE(waitForStream(c, 10).size(), 10u);

  // ...and so must be re-opening an already open client:
  FakeTcpDevice dev2(casterHandler());
  ASSERT_TRUE(dev2.isReady());
  good.port = dev2.port();
  ASSERT_TRUE(c.open(good, err)) << err;
  EXPECT_GE(waitForStream(c, 10).size(), 10u);
  c.close();
  c.close();  // closing twice is harmless
}

TEST(CNTRIPClient, RetrieveListOfMountpoints)
{
  // A source table with blank fields, CRLF line ends and lines to be skipped:
  const std::string table =
      "SOURCETABLE 200 OK\r\n"
      "Server: test\r\n"
      "Content-Type: text/plain\r\n"
      "\r\n"
      "CAS;caster.example.org;2101;Example;Op;0;ESP;40.0;-3.0;\r\n"
      "STR;MNT1;City1;RTCM 3.2;1004(1),1006(10);2;GPS+GLO;NET1;ESP;40.50;190.00;1;0;"
      "SoftGen;none;B;N;9600;misc info\r\n"
      "STR;MNT2;;RTCM 3;;2;GPS;NET2;FRA;-10.25;5.00;0;1;;;N;Y;4800;\r\n"
      "STR;too;few;fields\r\n"
      "ENDSOURCETABLE\r\n";
  FakeTcpDevice dev([table](const std::string&) { return table; }, {}, true /*close*/);
  ASSERT_TRUE(dev.isReady());

  CNTRIPClient::TListMountPoints lst;
  std::string err;
  ASSERT_TRUE(CNTRIPClient::retrieveListOfMountpoints(lst, err, "127.0.0.1", dev.port())) << err;
  ASSERT_EQ(lst.size(), 2u);

  const auto& m1 = lst.front();
  EXPECT_EQ(m1.mountpoint_name, "MNT1");
  EXPECT_EQ(m1.id, "City1");
  EXPECT_EQ(m1.format, "RTCM 3.2");
  EXPECT_EQ(m1.carrier, 2);
  EXPECT_EQ(m1.nav_system, "GPS+GLO");
  EXPECT_EQ(m1.country_code, "ESP");
  EXPECT_NEAR(m1.latitude, 40.5, 1e-9);
  EXPECT_NEAR(m1.longitude, -170.0, 1e-6);  // wrapped to [-180,180]
  EXPECT_TRUE(m1.needs_nmea);
  EXPECT_FALSE(m1.net_ref_stations);
  EXPECT_EQ(m1.generator_model, "SoftGen");
  EXPECT_EQ(m1.compr_encryp, "none");
  EXPECT_EQ(m1.authentication, 'B');
  EXPECT_FALSE(m1.pay_service);
  EXPECT_EQ(m1.stream_bitspersec, 9600);
  EXPECT_EQ(m1.extra_info, "misc info");

  // The second one has blank fields, which must not shift the following ones:
  const auto& m2 = *std::next(lst.begin());
  EXPECT_EQ(m2.mountpoint_name, "MNT2");
  EXPECT_TRUE(m2.id.empty());
  EXPECT_TRUE(m2.format_details.empty());
  EXPECT_EQ(m2.country_code, "FRA");
  EXPECT_NEAR(m2.latitude, -10.25, 1e-9);
  EXPECT_FALSE(m2.needs_nmea);
  EXPECT_TRUE(m2.net_ref_stations);
  EXPECT_TRUE(m2.pay_service);
  EXPECT_EQ(m2.stream_bitspersec, 4800);

  // No server:
  EXPECT_FALSE(CNTRIPClient::retrieveListOfMountpoints(lst, err, "127.0.0.1", 18799));
  EXPECT_TRUE(lst.empty());
}
