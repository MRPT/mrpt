/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

//INCLUDE
#include "hwdrivers-precomp.h"    // Precompiled headers

#include <mrpt/hwdrivers/CLMS151Eth_10110.h>
#include <mrpt/system/string_utils.h>

#include <iostream>
#include <sstream>
#include <string.h>

#define APPERTURE 4.712385  // in radian <=> 270

using namespace mrpt;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt::hwdrivers;
using namespace mrpt::poses;
using namespace mrpt::obs;
using namespace std;

//CODE
IMPLEMENTS_GENERIC_SENSOR(CLMS151Eth, mrpt::hwdrivers)

/** Default Lidar IP: 192.168.0.1
 *  Default IP_port: 2111
 *  Maximum Range: 10 Meters
 *  Default SensorPose: Depend on robot(0, 270, 105)mm, orientation(0, 21.25, 0)
 */
CLMS151Eth::CLMS151Eth(string _ip, unsigned int _port)
    : m_ip(_ip),
      m_port(_port),
      m_client(), m_turnedOn(false),
      m_cmd(),
      m_connected(false),
      m_sensorPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
      m_maxRange(10.0),
      m_beamApperture(.25 * M_PI / 180.0)
{
  setVerbosityLevel(mrpt::utils::LVL_DEBUG);
}

CLMS151Eth::~CLMS151Eth()
{
  if (m_connected) m_client.close();
  // delete m_client;
  // delete m_sensorPose;
}

void CLMS151Eth::initialize()
{
  if (!checkIsConnected())
  {
    THROW_EXCEPTION(
          "Cannot connect to SICK Tim561 Ethernet Sensor check your configuration" 
          "file.");
  }
  turnOn();
}

void CLMS151Eth::loadConfig_sensorSpecific(
      const mrpt::utils::CConfigFileBase& configSource,
      const std::string& iniSection)
{
  C2DRangeFinderAbstract::loadCommonParams(configSource, iniSection);
  float pose_x, pose_y, pose_z, pose_yaw, pose_pitch, pose_roll;

  pose_x = configSource.read_float(iniSection, "pose_x", 0, false);
  pose_y = configSource.read_float(iniSection, "pose_y", 0, false);
  pose_z = configSource.read_float(iniSection, "pose_z", 0, false);
  pose_yaw = configSource.read_float(iniSection, "pose_yaw", 0, false);
  pose_pitch = configSource.read_float(iniSection, "pose_pitch", 0, false);
  pose_roll = configSource.read_float(iniSection, "pose_roll", 0, false);
  m_ip = configSource.read_string(iniSection, "ip_address", "192.168.0.1", false);
  m_port = configSource.read_int(iniSection, "TCP_port", 2111, false);
  m_process_rate = configSource.read_int(iniSection, string("process_rate"), 15, false);
  m_sensorLabel = configSource.read_string(iniSection, "sensorLabel", "SICK", false);
  m_sensorPose = CPose3D(pose_x, pose_y, pose_z, DEG2RAD(pose_yaw), DEG2RAD(pose_pitch), DEG2RAD(pose_roll));
}

bool CLMS151Eth::checkIsConnected(void)
{
  if (m_connected)
  {
    return true;
  }
  else
  {
    try
    {
      m_client.connect(m_ip, m_port);
    }
    catch(std::exception& e)
    {
      MRPT_LOG_ERROR_FMT(
            "[LMS151Eth] ERROR TRYING TO OPEN Ethernet DEVICE:\n%s",
            e.what());
      return false;
    }
  }
  m_connected = true;
  return true;
}

bool CLMS151Eth::rebootDev()
{
  {
    // Set Maintenance access mode to allow reboot to be sent
    char msg[] = {"sMN SetAccessMode 03 F4724744"};
    char msgIn[100];
    sendCommand(msg);

    size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
    msgIn[read - 1] = 0;
    MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
    MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

    if (!read )
    {
      MRPT_LOG_ERROR("SOPAS - Error setting access mode, unexpected response"); 
      return false;
    }
  }
  {
    // Send reboot command
    char msg[] = {"sMN mSCreboot"};
    char msgIn[100];
    sendCommand(msg);

    size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
    msgIn[read - 1] = 0;
    MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
    MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

    if (!read )
    {
      MRPT_LOG_ERROR("SOPAS - Error rebootting scanner, unexpected response.");
      return false;
    }
  }
  return true;
}

bool CLMS151Eth::turnOff()
{
  if (m_client.isConnected()) m_client.close();
  m_connected = false;
  m_turnedOn = false;
  return true;
}

bool CLMS151Eth::turnOn()
{
  /** From the SICK LMS151 datasheet:
   * * 1. Login: "sMN SetAccessMode 03 F4724744"
   * *    -wait answer: "sAN SetAccessMode 1"
   * * 2. Set Freq and Resolution: "sMN mLMPsetscancfg +1500 0 1800000"
   * *    -wait answer(0-180): "sAN mLMPsetscancfg 0 +1500 0 1800000"
   * * 3. Start measurement: "sMN LMCstartmeas"
   * *    -wait answer(0/1 yes/no): "sAN LMCstartmeas 0"
   * * 4. Stop measurement: "sMN LMCstopmeas"
   * *    -wait answer(0/1 yes/no): "sAN LMCstopmeas 0"
   *
   *
   * * z. Configure scandata context: "sWN LMDscandatacfg"
   * *    -wait answer
   *
   * * x. Read Freq and Angular:
   * *    -Get params: "sRN LMPscancfg"
   * *    -Read params(Freq, Sect, Resolution): "sRA LMPscancfg 1500 0 1800000"
   */
  if (checkIsConnected())
  {
    try
    {
      {
        /** Login LMS151 
         */
        char msg[] = {"sMN SetAccessMode 03 F4724744"};
        char msgIn[100];
        sendCommand(msg);

        size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
        msgIn[read - 1] = 0;
        MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
        MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

        if (!read )
        {
          return false;
        }
      }
      {
        /** getScanCfg
         */
        char msg[] = {"sRN LMPscancfg"};
        char msgIn[100];
        sendCommand(msg);

        size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
        msgIn[read - 1] = 0;
        MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
        MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

        if (!read )
        {
          return false;
        }
      }
      
      {
        /** getScanOutRange
         */
        char msg[] = {"sRN LMPoutputRange"};
        char msgIn[100];
        sendCommand(msg);

        size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
        msgIn[read - 1] = 0;
        MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
        MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

        if (!read )
        {
          return false;
        }
      }

      {
        /** queryStatus
         */
        char msg[] = {"sRN STlms"};
        char msgIn[100];
        sendCommand(msg);

        size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
        msgIn[read - 1] = 0;
        MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
        MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

        if (!read )
        {
          return false;
        }
      }
      {
        /** startDevice
         */
        char msg[] = {"sMN Run"};
        char msgIn[100];
        sendCommand(msg);

        size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
        msgIn[read - 1] = 0;
        MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
        MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

        if (!read )
        {
          return false;
        }
      }
      //{
      //  /** scanContinue
      //   */
      //  char msg[] = {"sEN LMDscandata 1"};
      //  char msgIn[100];
      //  sendCommand(msg);

      //  size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
      //  msgIn[read - 1] = 0;
      //  MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
      //  MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

      //  if (!read )
      //  {
      //    return false;
      //  }
      //}


     // {
     //   /** Login LMS151 
     //    */
     //   char msg[] = {"sMN SetAccessMode 03 F4724744"};
     //   char msgIn[100];
     //   sendCommand(msg);

     //   size_t  read = m_client.readAsync(msgIn, 100, 1000, 1000);
     //   msgIn[read - 1] = 0;
     //   MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
     //   MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

     //   if (!read )
     //   {
     //     return false;
     //   }
     // }
     // while(true);
        /** Set Freq and Resolution
         */
     // {
     //   char msg[] = {"sMN mLMPsetscancfg +1500 +1 0 1800000"};
     //   char msgIn[100];
     //   sendCommand(msg);

     //   size_t read = m_client.readAsync(msgIn, 100, 1000, 1000);
     //   msgIn[read - 1] = 0;
     //   MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
     //   MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());

     //    if (!read)
     //   {
     //     return false;
     //   }
     // }

        /** Set scandatacfg
         */
     // {
     //   char msg[] = {"sWN LMDscandatacfg 01 00 0 1 0 00 00 0 0 0 0 +1"};
     //   char msgIn[100];
     //   sendCommand(msg);

     //   size_t read =m_client.readAsync(msgIn, 100, 1000, 1000);
     //   msgIn[read - 1] = 0;
     //   MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
     //   MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());
     //   if (!read){
     //     return false;
     //   }
     // }

     // {
     //    /** Start measurement
     //    */
     //   char msg[] = {"sMN LMCstartmeas"};
     //   char msgIn[100];
     //   sendCommand(msg);
     //   size_t read = m_client.readAsync(msgIn, 100, 1000, 1000);
     //   msgIn[read - 1] = 0;
     //   MRPT_LOG_DEBUG_FMT("read : %u\n", (unsigned int)read);
     //   MRPT_LOG_DEBUG_FMT("message : %s\n", string(&msgIn[1]).c_str());
     //   if (!read)
     //   {
     //     return false;
     //   }
     // }
      m_turnedOn = true;
    }
    catch (std::exception& e){
      MRPT_LOG_ERROR_FMT("%s", e.what());
      return false;
    }
  }
  else
  {
    return false;
  }
  return true;
}

void CLMS151Eth::sendCommand(const char* cmd)
{
  generateCmd(cmd);
  if (!m_cmd.empty())
  {
    m_client.writeAsync(&m_cmd[0], m_cmd.size());
  }
}

void CLMS151Eth::generateCmd(const char* cmd)
{
  if( strlen(cmd) > 995 )
  {
    MRPT_LOG_ERROR("Error: command is too long.");
    return;
  }
  //m_cmd = format("%c%s%c", 0x02, cmd, 0x03);
  m_cmd = format("%c%s%c%c", 0x02, cmd, 0x03, 0);

}

bool CLMS151Eth::decodeScan(char* buff, CObservation2DRangeScan& outObservation)
{
  char* next;
  unsigned int idx = 0;
  unsigned int scanCount = 0;
  char* tmp;

  next = strtok(buff, " ", &tmp);

  while (next && scanCount == 0)
  {
    switch (++idx)
    {
      case 1:
        // If no "sRA" and also no "sSN", return false reading
        if (strncmp(&next[1], "sRA", 3) && strncmp(&next[1], "sSN", 3))
        {
          MRPT_LOG_DEBUG("No sRA or sSN");
          return false;
        }
        break;
      case 2:
        if (strcmp(next, "LMDscandata"))
        {
          MRPT_LOG_DEBUG("No LMDscandata");
          return false;
        }
        break;
      case 6:
        if (strcmp(next, "1"))
        {
          MRPT_LOG_DEBUG("Laser is ready");
        }
        else if (strcmp(next, "0"))
        {
          MRPT_LOG_DEBUG("Laser is busy");
        }
        else
        {
          MRPT_LOG_DEBUG("Laser reports error");
          rebootDev();
        }
        break;
      case 21:
        if (strcmp(next, "DIST1"))
        {
          THROW_EXCEPTION("LMS151 is not configured to send distances");
          return false;
        }
        MRPT_LOG_DEBUG("Distance : OK\n");
        break;
      case 26:
        scanCount = strtoul(next, nullptr, 16);
        MRPT_LOG_DEBUG_FMT("Scan Count : %d\n", scanCount);
        break;
      default:
        break;
    }
    next = strtok(nullptr, " ", &tmp);
  }
  outObservation.aperture = (float)APPERTURE;
  outObservation.rightToLeft = false;
  outObservation.stdError = 0.012f;
  outObservation.sensorPose = m_sensorPose;
  outObservation.beamAperture = m_beamApperture;
  outObservation.maxRange = m_maxRange;
  outObservation.timestamp = mrpt::system::getCurrentTime();
  outObservation.sensorLabel = m_sensorLabel;

  outObservation.resizeScan(scanCount);
  unsigned int i;
  for(i = 0; i < scanCount && next; i++, next =strtok(nullptr, " ", &tmp))
  {
    outObservation.setScanRange(i, double(strtoul(next, nullptr, 16)) / 1000.0);
    outObservation.setScanRangeValidity(i, outObservation.getScanRange(i) <= outObservation.maxRange);
  }
  outObservation.resizeScan(i);
  return i >= scanCount;
}

void CLMS151Eth::doProcessSimple(
      bool& outThereIsObservation, CObservation2DRangeScan& outObservation,
      bool& hardwareError)
{
  if (!m_turnedOn){
    hardwareError = true;
    outThereIsObservation = false;
    return;
  }
  hardwareError = false;

  char msg[] = {"sRN LMDscandata"};
  sendCommand(msg);
  char buffIn[16 * 1024];

  m_client.readAsync(buffIn, sizeof(buffIn), 40, 40);

  if (decodeScan(buffIn, outObservation))
  {
    // Filter:
    C2DRangeFinderAbstract::filterByExclusionAreas(outObservation);
    C2DRangeFinderAbstract::filterByExclusionAngles(outObservation);
    // Do show preview:
    C2DRangeFinderAbstract::processPreview(outObservation);
    MRPT_LOG_DEBUG("doProcessSimple Show");

    outThereIsObservation = true;
    hardwareError = false;
  }
  else
  {
    hardwareError = true;
    outThereIsObservation = false;
    MRPT_LOG_ERROR("doProcessSimple failed\n");
  }
}

void CLMS151Eth::doProcess(){
    CObservation2DRangeScan::Ptr obs = mrpt::make_aligned_shared<CObservation2DRangeScan>();
  try
  {
    bool isThereObservation, hwError;
    doProcessSimple(isThereObservation, *obs, hwError);
    if (hwError){
      m_state = ssError;
      MRPT_LOG_DEBUG("state Error");
    }
    else
    {
      m_state = ssWorking;
      MRPT_LOG_DEBUG("state working");
    }
    // if at least one data have been sensed:
    if (isThereObservation)
    {
      appendObservation(obs);
    }
  }
  catch (...)
  {
    m_state = ssError;
    THROW_EXCEPTION("No observation received from the Phidget board!");
  }
}


/** A method to set the sensor pose on the robot.
 */
void CLMS151Eth::setSensorPose(const CPose3D& _pose) { m_sensorPose = _pose; }
