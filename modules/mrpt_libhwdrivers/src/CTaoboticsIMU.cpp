/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "hwdrivers-precomp.h"  // Precompiled headers
//
#include <mrpt/core/bit_cast.h>
#include <mrpt/core/reverse_bytes.h>
#include <mrpt/hwdrivers/CTaoboticsIMU.h>
#include <mrpt/obs/CObservationIMU.h>

#include <chrono>
#include <iostream>
#include <thread>

IMPLEMENTS_GENERIC_SENSOR(CTaoboticsIMU, mrpt::hwdrivers)

using namespace mrpt::comms;
using namespace mrpt::obs;
using namespace mrpt::hwdrivers;

CTaoboticsIMU::CTaoboticsIMU()
{
  m_state = ssInitializing;
  m_sensorLabel = "IMU";
}

CTaoboticsIMU::~CTaoboticsIMU()
{
  if (m_serialPort) m_serialPort->close();
}

void CTaoboticsIMU::setSerialPort(const std::string& serialPort)
{
  ASSERTMSG_(!m_serialPort, "setSerialPort() can be called only before initialize()");

  m_com_port = serialPort;
}

void CTaoboticsIMU::setSerialBaudRate(int rate)
{
  ASSERTMSG_(!m_serialPort, "setSerialBaudRate() can be called only before initialize()");

  m_baudRate = rate;
}

void CTaoboticsIMU::doProcess()
{
  using namespace std::chrono_literals;

  ASSERTMSG_(m_activeParser, "initialize() must be called first");
  ASSERT_(m_serialPort);

  if (m_state == ssError)
  {
    std::this_thread::sleep_for(200ms);
    initialize();
  }

  if (m_state == ssError) return;

  // try to read and parse a frame from the serial port:
  std::vector<uint8_t> buf(m_rx_buffer.available());

  try
  {
    const auto nRead = m_serialPort->ReadBufferImmediate(buf.data(), buf.size());
    m_rx_buffer.push_many(buf.data(), nRead);
  }
  catch (const std::exception& e)
  {
    std::cerr << "[CTaobotics] Error readingopen serial port " << m_com_port << std::endl;
    m_state = ssError;
  }

  // Parse:
  ASSERT_(m_activeParser);
  const auto allObs = m_activeParser(this, m_rx_buffer);

  // and send observations out, if any:
  for (const auto& obs : allObs) this->appendObservation(obs);
}

void CTaoboticsIMU::initialize()
{
  if (m_sensorModel == "hfi-b6")
    m_activeParser = &CTaoboticsIMU::parser_hfi_b6;
  else if (m_sensorModel == "hfi-a9")
    m_activeParser = &CTaoboticsIMU::parser_hfi_a9;
  else
  {
    THROW_EXCEPTION_FMT("Unknown sensor model: '%s'", m_sensorModel.c_str());
  }

  if (m_verbose)
    std::cout << "[CTaoboticsIMU] Opening port: " << m_com_port << " at " << m_baudRate
              << " bauds.\n";

  try
  {
    m_serialPort = std::make_unique<CSerialPort>(m_com_port);
    ASSERT_(m_serialPort && m_serialPort->isOpen());

    m_serialPort->setConfig(m_baudRate);
    m_serialPort->setTimeouts(1, 1, 1, 1, 1);

    m_serialPort->purgeBuffers();
    m_rx_buffer.clear();

    m_state = ssWorking;
  }
  catch (const std::exception& e)
  {
    std::cerr << "[CTaobotics] Can't open serial port " << m_com_port << std::endl;
    m_state = ssError;
  }
}

void CTaoboticsIMU::loadConfig_sensorSpecific(
    const mrpt::config::CConfigFileBase& c, const std::string& s)
{
  m_sensorPose.setFromValues(
      c.read_float(s, "pose_x", 0, false), c.read_float(s, "pose_y", 0, false),
      c.read_float(s, "pose_z", 0, false), DEG2RAD(c.read_float(s, "pose_yaw", 0, false)),
      DEG2RAD(c.read_float(s, "pose_pitch", 0, false)),
      DEG2RAD(c.read_float(s, "pose_roll", 0, false)));

  m_com_port = c.read_string(s, "serialPort", m_com_port);
  m_sensorModel = c.read_string(s, "sensorModel", m_sensorModel);
}

std::vector<mrpt::obs::CObservation::Ptr> CTaoboticsIMU::parser_hfi_b6(
    mrpt::containers::circular_buffer<uint8_t>& buf) const
{
  std::vector<mrpt::obs::CObservation::Ptr> out;

  // Parse:
  // Header must be: "0x55 0x5*".
  // But we must wait for the first "0x55 0x51" to ensure synchronization
  // of all IMU data fields in one observation:
  while (buf.size() >= 11)
  {
    if (buf.peek(0) == 0x55 && buf.peek(1) == 0x51)
    {
      break;
    }
    else
    {
      // discard byte:
      buf.pop();
    }
  }

  // Actual processing, of one or more observations:
  mrpt::obs::CObservationIMU::Ptr obs;

  while (buf.size() >= 11)
  {
    if (buf.peek(0) == 0x55 && (buf.peek(1) & 0x50) == 0x50)
    {
      std::array<uint8_t, 11> frame;
      buf.pop_many(frame.data(), frame.size());

      // Parse this frame. Each frame seems to be fixed size = 11
      // bytes.
      //
      // Frame field :  0x55 | TYPE |  D0  |  D1  |  D2  |  D3  |CHKSUM
      //  Byte index :   0   |  1   | 2  3 | 4  5 | 6  7 | 8  9 |   10
      //
      // Type:
      // 0x51: acceleration, 0x52: angular velocity, 0x53: euler
      // angles
      //
      // TODO: Checksum

      uint16_t data[4];
      for (int i = 0; i < 4; i++)
      {
        data[i] = frame[2 + 2 * i] | (frame[2 + 2 * i + 1] << 8);
        mrpt::toNativeEndianness(data[i]);
      }

      switch (frame[1])
      {
        case 0x51:
        {
          if (!obs) obs = std::make_shared<CObservationIMU>();
          const double accx = data[0] / 32768.0 * 16 * -9.8;
          const double accy = data[1] / 32768.0 * 16 * -9.8;
          const double accz = data[2] / 32768.0 * 16 * -9.8;
          printf("acc: %f %f %f\n", accx, accy, accz);
        }
        break;
        case 0x52:
        {
          if (!obs) obs = std::make_shared<CObservationIMU>();
          //
        }
        break;
        case 0x53:
        {
          // and send out one obs:
          if (!obs) obs = std::make_shared<CObservationIMU>();

          // get euler angle:

          // send out:
          obs->timestamp = mrpt::Clock::now();

          obs->sensorPose = m_sensorPose;
          obs->sensorLabel = m_sensorLabel;

          out.push_back(obs);
          obs.reset();
        }
        break;
        default:
          fprintf(
              stderr,
              "[CTaoboticsIMU] Received frame with unknown data type "
              "id=0x%02X\n",
              frame[1]);
          break;
      }
    }
    else
    {
      // discard byte:
      buf.pop();
    }
  }

  return out;
}

std::vector<mrpt::obs::CObservation::Ptr> CTaoboticsIMU::parser_hfi_a9(
    mrpt::containers::circular_buffer<uint8_t>& buf) const
{
  using namespace mrpt::obs;

  std::vector<mrpt::obs::CObservation::Ptr> out;

  // Parse. See .h for data frame format
  while (buf.size() >= 49 + 25)
  {
    if (buf.peek(0) == 0xAA && buf.peek(1) == 0x55 && buf.peek(2) == 0x2c)
    {
      break;
    }
    else
    {
      // discard byte:
      buf.pop();
    }
  }

  // Actual processing, of one or more observations:
  mrpt::obs::CObservationIMU::Ptr obs;

  while (buf.size() >= 49 + 25)
  {
    if (buf.peek(0) == 0xAA && buf.peek(1) == 0x55 && buf.peek(2) == 0x2c)
    {
      std::array<uint8_t, 49> frame;
      buf.pop_many(frame.data(), frame.size());

#if 0
			const uint32_t hwTimestamp = (frame[7 + 3] << 24) |	 //
				(frame[7 + 2] << 16) |	//
				(frame[7 + 1] << 8) |  //
				(frame[7 + 0] << 0);
#endif

      float data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
      for (int i = 0; i < 9; i++)
      {
        // Fixed format: little endian for this sensor.
        const uint32_t d =                   //
            (frame[11 + 4 * i + 3] << 24) |  //
            (frame[11 + 4 * i + 2] << 16) |  //
            (frame[11 + 4 * i + 1] << 8) |   //
            (frame[11 + 4 * i + 0] << 0);
        data[i] = bit_cast<float>(d);
      }

      if (!obs)
      {
        obs = std::make_shared<CObservationIMU>();
        obs->sensorPose = m_sensorPose;
        obs->sensorLabel = m_sensorLabel;
      }

      const float wx = data[0], wy = data[1], wz = data[2];
      const float ax = data[3], ay = data[4], az = data[5];
      const float mx = data[6], my = data[7], mz = data[8];

      // send out:
      obs->timestamp = mrpt::Clock::now();

      obs->set(IMU_X_ACC, -9.8f * ax);
      obs->set(IMU_Y_ACC, -9.8f * ay);
      obs->set(IMU_Z_ACC, -9.8f * az);

      obs->set(IMU_WZ, wz);
      obs->set(IMU_WY, wy);
      obs->set(IMU_WX, wx);

      obs->set(IMU_MAG_X, mx);
      obs->set(IMU_MAG_Y, my);
      obs->set(IMU_MAG_Z, mz);

      // do not send out the obs yet, we want to also add the ypr angles
      // in the next frame:
    }
    else
    {
      // discard byte:
      buf.pop();
    }

    if (buf.peek(0) == 0xAA && buf.peek(1) == 0x55 && buf.peek(2) == 0x14)
    {
      std::array<uint8_t, 25> frame;
      buf.pop_many(frame.data(), frame.size());

#if 0
			const uint32_t hwTimestamp = (frame[7 + 3] << 24) |	 //
				(frame[7 + 2] << 16) |	//
				(frame[7 + 1] << 8) |  //
				(frame[7 + 0] << 0);
#endif

      float data[3] = {0, 0, 0};
      for (int i = 0; i < 3; i++)
      {
        // Fixed format: little endian for this sensor.
        const uint32_t d =                   //
            (frame[11 + 4 * i + 3] << 24) |  //
            (frame[11 + 4 * i + 2] << 16) |  //
            (frame[11 + 4 * i + 1] << 8) |   //
            (frame[11 + 4 * i + 0] << 0);
        data[i] = bit_cast<float>(d);
      }

      if (!obs)
      {
        obs = std::make_shared<CObservationIMU>();
        obs->sensorPose = m_sensorPose;
        obs->sensorLabel = m_sensorLabel;
      }

      const float roll = mrpt::DEG2RAD(data[0]), pitch = mrpt::DEG2RAD(-data[1]),
                  yaw = mrpt::DEG2RAD(-data[2]);

      // send out:
      obs->timestamp = mrpt::Clock::now();

      mrpt::math::CQuaternionDouble q;
      mrpt::poses::CPose3D::FromYawPitchRoll(yaw, pitch, roll).getAsQuaternion(q);

      obs->set(IMU_ORI_QUAT_W, q.r());
      obs->set(IMU_ORI_QUAT_X, q.x());
      obs->set(IMU_ORI_QUAT_Y, q.y());
      obs->set(IMU_ORI_QUAT_Z, q.z());

      out.push_back(obs);
      obs.reset();
    }
    else
    {
      // discard byte:
      buf.pop();
    }
  }

  return out;
}
