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
#include <mrpt/comms/CClientTCPSocket.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/hwdrivers/CImpinjRFID.h>
#include <mrpt/obs/CObservationRFID.h>

#include <atomic>
#include <chrono>
#include <thread>

using namespace mrpt::hwdrivers;

namespace
{
/** One tag report as sent by the reader interface program: a fixed 34-byte
 * block "ANT_PORT EPC RX_PWR" padded with zeros. */
std::string tagMessage(const std::string& text)
{
  std::string s = text;
  s.resize(34, '\0');
  return s;
}

/** Plays the role of the external driver: it connects to the sensor, and
 * answers every "OBS" command with the queued tag messages. */
class FakeReaderProgram
{
 public:
  FakeReaderProgram(unsigned short port, std::vector<std::string> tagsPerObs) :
      m_port(port), m_tags(std::move(tagsPerObs)), m_thread([this]() { run(); })
  {
  }
  ~FakeReaderProgram()
  {
    m_stop = true;
    m_thread.join();
  }
  FakeReaderProgram(const FakeReaderProgram&) = delete;
  FakeReaderProgram& operator=(const FakeReaderProgram&) = delete;

  bool gotEndCommand() const { return m_gotEnd; }

 private:
  void run()
  {
    mrpt::comms::CClientTCPSocket sock;
    // The sensor starts listening a moment after initialize() is called:
    for (int i = 0; i < 100 && !m_stop; i++)
    {
      try
      {
        sock.connect("127.0.0.1", m_port);
        break;
      }
      catch (const std::exception&)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    if (!sock.isConnected())
    {
      return;
    }
    char buf[10];
    while (!m_stop)
    {
      if (sock.readAsync(buf, 10, 50, 50) < 3)
      {
        continue;
      }
      const std::string cmd(buf, 3);
      if (cmd == "OBS")
      {
        for (const auto& t : m_tags)
        {
          const std::string msg = tagMessage(t);
          sock.writeAsync(msg.data(), msg.size());
        }
      }
      else if (cmd == "END")
      {
        m_gotEnd = true;
        return;
      }
    }
  }

  unsigned short m_port;
  std::vector<std::string> m_tags;
  std::atomic<bool> m_stop{false};
  std::atomic<bool> m_gotEnd{false};
  std::thread m_thread;
};

mrpt::config::CConfigFileMemory rfidConfig(unsigned short port)
{
  mrpt::config::CConfigFileMemory cfg;
  cfg.write("RFID", "local_IP", "127.0.0.1");
  cfg.write("RFID", "reader_name", "reader1");
  cfg.write("RFID", "listen_port", std::to_string(port));
  // The "driver executable": something that exists everywhere and succeeds.
  cfg.write("RFID", "driver_path", "true");
  return cfg;
}
}  // namespace

TEST(CImpinjRFID, ReadsTagsFromTheReaderProgram)
{
  // (each test uses its own port: a just-closed one can linger)
  FakeReaderProgram reader(
      18790, {
                 "1 E2000016 -55.5", "2 E2000017 -60.25",
                 "3 BADMESSAGE",  // malformed: no power field, must be skipped
             });
  bool endSeen = false;
  {
    CImpinjRFID sensor;
    const auto cfg = rfidConfig(18790);
    sensor.loadConfig(cfg, "RFID");
    sensor.initialize();

    mrpt::obs::CObservationRFID obs;
    ASSERT_TRUE(sensor.getObservation(obs));
    ASSERT_EQ(obs.tag_readings.size(), 2u);
    EXPECT_EQ(obs.tag_readings[0].antennaPort, "1");
    EXPECT_EQ(obs.tag_readings[0].epc, "E2000016");
    EXPECT_NEAR(obs.tag_readings[0].power, -55.5, 1e-6);
    EXPECT_EQ(obs.tag_readings[1].epc, "E2000017");
    EXPECT_NEAR(obs.tag_readings[1].power, -60.25, 1e-6);

    // The same through the generic sensor interface:
    sensor.doProcess();
    const auto lst = sensor.getObservations();
    ASSERT_EQ(lst.size(), 1u);
    EXPECT_TRUE(std::dynamic_pointer_cast<mrpt::obs::CObservationRFID>(lst.begin()->second));

    sensor.closeReader();
    for (int i = 0; i < 100 && !reader.gotEndCommand(); i++)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    endSeen = reader.gotEndCommand();
  }
  EXPECT_TRUE(endSeen);
}

TEST(CImpinjRFID, NoTagsMeansNoObservation)
{
  FakeReaderProgram reader(18791, {});
  CImpinjRFID sensor;
  const auto cfg = rfidConfig(18791);
  sensor.loadConfig(cfg, "RFID");
  sensor.initialize();

  mrpt::obs::CObservationRFID obs;
  EXPECT_FALSE(sensor.getObservation(obs));
  sensor.doProcess();
  EXPECT_TRUE(sensor.getObservations().empty());
}

TEST(CImpinjRFID, DestructorWithoutInitializeIsHarmless)
{
  {
    CImpinjRFID sensor;
  }
  SUCCEED();
}
