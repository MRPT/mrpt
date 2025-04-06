/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "obs-precomp.h"  // Precomp header
//
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/system/filesystem.h>

#include <fstream>

using namespace std;
using namespace mrpt::obs;

// This must be added to any CSerializable class implementation file.
IMPLEMENTS_SERIALIZABLE(CObservationRotatingScan, CObservation, mrpt::obs)

using RotScan = CObservationRotatingScan;

mrpt::system::TTimeStamp RotScan::getOriginalReceivedTimeStamp() const
{
  return originalReceivedTimestamp;
}

namespace
{
void writeMatricesTo(const RotScan& me, mrpt::serialization::CArchive& out)
{
  out.WriteAs<uint16_t>(me.rangeImage.cols());
  out.WriteAs<uint16_t>(me.rangeImage.rows());
  if (!me.rangeImage.empty())
    out.WriteBufferFixEndianness(&me.rangeImage(0, 0), me.rangeImage.size());

  out.WriteAs<uint16_t>(me.intensityImage.cols());
  out.WriteAs<uint16_t>(me.intensityImage.rows());
  if (!me.intensityImage.empty())
    out.WriteBufferFixEndianness(&me.intensityImage(0, 0), me.intensityImage.size());

  out.WriteAs<uint16_t>(me.rangeOtherLayers.size());
  for (const auto& ly : me.rangeOtherLayers)
  {
    out << ly.first;
    ASSERT_EQUAL_(ly.second.cols(), me.columnCount);
    ASSERT_EQUAL_(ly.second.rows(), me.rowCount);
    out.WriteBufferFixEndianness(&ly.second(0, 0), ly.second.size());
  }

  out.WriteAs<uint16_t>(me.organizedPoints.cols());
  out.WriteAs<uint16_t>(me.organizedPoints.rows());
  for (const auto& pt : me.organizedPoints) out << pt;
}

void readMatricesFrom(RotScan& me, mrpt::serialization::CArchive& in)
{
  const auto nCols = in.ReadAs<uint16_t>(), nRows = in.ReadAs<uint16_t>();
  me.rangeImage.resize(nRows, nCols);
  if (!me.rangeImage.empty())
    in.ReadBufferFixEndianness(&me.rangeImage(0, 0), me.rangeImage.size());

  {
    const auto nIntCols = in.ReadAs<uint16_t>(), nIntRows = in.ReadAs<uint16_t>();
    me.intensityImage.resize(nIntRows, nIntCols);
    if (!me.intensityImage.empty())
      in.ReadBufferFixEndianness(&me.intensityImage(0, 0), me.intensityImage.size());
  }

  const auto nOtherLayers = in.ReadAs<uint16_t>();
  me.rangeOtherLayers.clear();
  for (size_t i = 0; i < nOtherLayers; i++)
  {
    std::string name;
    in >> name;
    auto& im = me.rangeOtherLayers[name];
    im.resize(nRows, nCols);
    in.ReadBufferFixEndianness(&im(0, 0), im.size());
  }

  const auto nIntCols = in.ReadAs<uint16_t>(), nIntRows = in.ReadAs<uint16_t>();
  me.organizedPoints.resize(nIntRows, nIntCols);
  for (auto& pt : me.organizedPoints) in >> pt;
}

}  // namespace

uint8_t RotScan::serializeGetVersion() const { return 0; }
void RotScan::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << timestamp << sensorLabel << rowCount << columnCount << rangeResolution << startAzimuth
      << azimuthSpan << sweepDuration << lidarModel << minRange << maxRange << sensorPose
      << originalReceivedTimestamp << has_satellite_timestamp;

  out.WriteAs<uint8_t>(m_externally_stored);

  if (isExternallyStored())
  {  //
    out << m_external_file;
  }
  else
  {
    writeMatricesTo(*this, out);
  }
}

void RotScan::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      in >> timestamp >> sensorLabel >> rowCount >> columnCount >> rangeResolution >>
          startAzimuth >> azimuthSpan >> sweepDuration >> lidarModel >> minRange >> maxRange >>
          sensorPose >> originalReceivedTimestamp >> has_satellite_timestamp;

      m_externally_stored = static_cast<ExternalStorageFormat>(in.ReadPOD<uint8_t>());

      if (isExternallyStored())
      {  // just read the file name
        in >> m_external_file;
        rangeImage.resize(0, 0);
        intensityImage.resize(0, 0);
        organizedPoints.resize(0, 0);
        rangeOtherLayers.clear();
      }
      else
      {
        m_external_file.clear();
        readMatricesFrom(*this, in);
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void RotScan::getDescriptionAsText(std::ostream& o) const
{
  CObservation::getDescriptionAsText(o);
  o << "Homogeneous matrix for the sensor 3D pose, relative to "
       "robot base:\n";
  o << sensorPose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>() << "\n"
    << sensorPose << endl;

  o << "lidarModel: " << lidarModel << "\n";
  o << "Range rows=" << rowCount << " cols=" << columnCount << "\n";
  o << "Range resolution=" << rangeResolution << " [meter]\n";
  o << "Has organized points=" << (!organizedPoints.empty() ? "YES" : "NO") << "\n";
  o << "Scan azimuth: start=" << mrpt::RAD2DEG(startAzimuth)
    << " span=" << mrpt::RAD2DEG(azimuthSpan) << "\n";
  o << "Sweep duration: " << sweepDuration << " [s]\n";
  o << mrpt::format("Sensor min/max range: %.02f / %.02f m\n", minRange, maxRange);
  o << "has_satellite_timestamp: " << (has_satellite_timestamp ? "YES" : "NO") << "\n";
  o << "originalReceivedTimestamp: " << mrpt::system::dateTimeToString(originalReceivedTimestamp)
    << " (UTC)\n";
}

void RotScan::fromVelodyne(const mrpt::obs::CObservationVelodyneScan& o)
{
  using Velo = mrpt::obs::CObservationVelodyneScan;
  using degree_cents = uint16_t;
  using gps_microsecs = uint32_t;

  MRPT_START

  // Reset:
  *this = CObservationRotatingScan();

  // Copy properties:
  has_satellite_timestamp = o.has_satellite_timestamp;
  originalReceivedTimestamp = o.originalReceivedTimestamp;
  timestamp = o.timestamp;
  sensorPose = o.sensorPose;
  sensorLabel = o.sensorLabel;
  minRange = o.minRange;
  maxRange = o.maxRange;

  // Convert ranges to range images:

  uint8_t model = 0;
  gps_microsecs last_pkt_tim = std::numeric_limits<gps_microsecs>::max();
  degree_cents last_pkt_az = 0;  // last azimuth

  // Azimuth (wrt sensor) at the beginning of one packet, to its timestamp:
  std::map<degree_cents, gps_microsecs> azimuth2timestamp;
  std::multiset<double> rotspeed;  // per packet estimated rot speed (deg/sec)

  // column count:
  const size_t num_lasers = o.calibration.laser_corrections.size();
  ASSERT_GT_(num_lasers, 2);
  rowCount = num_lasers;

  // row count:
  columnCount = Velo::SCANS_PER_BLOCK * o.scan_packets.size() * Velo::BLOCKS_PER_PACKET;

  const double timeBetweenLastTwoBlocks = 1e-6 * (o.scan_packets.rbegin()->gps_timestamp() -
                                                  (o.scan_packets.rbegin() + 1)->gps_timestamp());

  rangeImage.setZero(rowCount, columnCount);
  intensityImage.setZero(rowCount, columnCount);
  rangeOtherLayers.clear();
  rangeResolution = Velo::DISTANCE_RESOLUTION;
  azimuthSpan = 0;

  ASSERT_GE_(o.scan_packets.size(), 1);

  for (size_t pktIdx = 0; pktIdx < o.scan_packets.size(); pktIdx++)
  {
    const auto& pkt = o.scan_packets[pktIdx];

    model = pkt.velodyne_model_ID;

    const degree_cents pkt_azimuth = pkt.blocks[0].rotation();

    if (last_pkt_tim != std::numeric_limits<uint32_t>::max())
    {
      // Estimate rot speed:
      ASSERT_GT_(pkt.gps_timestamp(), last_pkt_tim);
      const double dT = 1e-6 * (pkt.gps_timestamp() - last_pkt_tim);
      const auto dAzimuth = 1e-2 * (pkt_azimuth - last_pkt_az);
      const auto estRotVel = dAzimuth / dT;
      rotspeed.insert(estRotVel);
    }
    last_pkt_tim = pkt.gps_timestamp();
    last_pkt_az = pkt_azimuth;

    azimuth2timestamp[pkt_azimuth] = pkt.gps_timestamp();

    // Accum azimuth span:
    if (pktIdx + 1 == o.scan_packets.size())
    {
      // last packet:
      // sanity checks: rot speed should be fairly stable:
      const double maxRotSpeed = *rotspeed.rbegin(), minRotSpeed = *rotspeed.begin();
      ASSERT_GT_(maxRotSpeed, 0);
      ASSERT_LT_((maxRotSpeed - minRotSpeed) / maxRotSpeed, 0.01);

      // Median speed:
      const double rotVel_degps = [&]()
      {
        auto it = rotspeed.begin();
        std::advance(it, rotspeed.size() / 2);
        return *it;
      }();

      azimuthSpan += mrpt::DEG2RAD(rotVel_degps * timeBetweenLastTwoBlocks);
    }
    else
    {
      // non-last packet:
      const double curAng = 0.01 * o.scan_packets[pktIdx].blocks[0].rotation();
      const double nextAng = 0.01 * o.scan_packets[pktIdx + 1].blocks[0].rotation();

      const double incrAng = mrpt::math::angDistance(mrpt::DEG2RAD(curAng), mrpt::DEG2RAD(nextAng));
      azimuthSpan += incrAng;
    }

    // Process each block in this packet:
    for (int block = 0; block < Velo::BLOCKS_PER_PACKET; block++)
    {
      const int dsr_offset = (pkt.blocks[block].header() == Velo::LOWER_BANK) ? 32 : 0;
      const bool block_is_dual_strongest_range =
          (pkt.laser_return_mode == Velo::RETMODE_DUAL && ((block & 0x01) != 0));
      const bool block_is_dual_last_range =
          (pkt.laser_return_mode == Velo::RETMODE_DUAL && ((block & 0x01) == 0));

      for (int dsr = 0, k = 0; dsr < Velo::SCANS_PER_FIRING; dsr++, k++)
      {
        if (!pkt.blocks[block].laser_returns[k].distance())  // Invalid return?
          continue;

        const auto rawLaserId = static_cast<uint8_t>(dsr + dsr_offset);
        uint8_t laserId = rawLaserId;

        // Detect VLP-16 data and adjust laser id if necessary
        // bool firingWithinBlock = false;
        if (num_lasers == 16)
        {
          if (laserId >= 16)
          {
            laserId -= 16;
            // firingWithinBlock = true;
          }
        }

        ASSERT_LT_(laserId, num_lasers);
        const auto& calib = o.calibration.laser_corrections[laserId];

        // In dual return, if the distance is equal in both ranges,
        // ignore one of them:

        const auto distance =
            pkt.blocks[block].laser_returns[k].distance() +
            static_cast<uint16_t>(calib.distanceCorrection / Velo::DISTANCE_RESOLUTION);

        const auto columnIdx = [&]()
        {
          switch (num_lasers)
          {
            case 16:
            case 32:
            case 64:
            {
              int c = (dsr + block * Velo::SCANS_PER_BLOCK + pktIdx * Velo::SCANS_PER_PACKET) /
                      num_lasers;
              if (pkt.laser_return_mode == Velo::RETMODE_DUAL) c /= 2;
              return c;
            }
            default:
            {
              THROW_EXCEPTION("Error: unhandled LIDAR model!");
            }
          };
        }();

        ASSERT_LT_(columnIdx, columnCount);
        if (pkt.laser_return_mode != Velo::RETMODE_DUAL || block_is_dual_strongest_range)
        {
          // Regular range, or strongest in multi return mode:
          rangeImage(laserId, columnIdx) = distance;

          // Intensity:
          intensityImage(laserId, columnIdx) = pkt.blocks[block].laser_returns[k].intensity();
        }
        else if (block_is_dual_last_range)
        {
          // Regular range, or strongest in multi return mode:
          auto& r = rangeOtherLayers["STRONGEST"];
          // 1st time init:
          if (static_cast<size_t>(r.rows()) != num_lasers) r.setZero(rowCount, columnCount);

          r(laserId, columnIdx) = distance;
        }

      }  // end for k,dsr=[0,31]
    }    // end for each block [0,11]
  }

  // Start and end azimuth:
  startAzimuth = mrpt::DEG2RAD(o.scan_packets.begin()->blocks[0].rotation() * 1e-2);

  const auto microsecs_1st_pkt = o.scan_packets.begin()->gps_timestamp();
  const auto microsecs_last_pkt = o.scan_packets.rbegin()->gps_timestamp();
  sweepDuration = 1e-6 * (microsecs_last_pkt - microsecs_1st_pkt) + timeBetweenLastTwoBlocks;

  MRPT_TODO("populate organizedPoints");

  // Decode model byte:
  switch (model)
  {
    case 0x21:
      lidarModel = "HDL-32E";
      break;
    case 0x22:
      lidarModel = "VLP-16";
      break;
    default:
      lidarModel = "Unknown";
      break;
  };

  MRPT_END
}

void RotScan::fromScan2D(const mrpt::obs::CObservation2DRangeScan& o)
{
  MRPT_START

  // Reset:
  *this = CObservationRotatingScan();

  // Copy properties:
  this->has_satellite_timestamp = false;
  this->timestamp = o.timestamp;
  this->sensorPose = o.sensorPose;
  this->sensorLabel = o.sensorLabel;
  this->maxRange = o.maxRange;

  // Convert ranges to range images:
  this->rowCount = 1;
  this->columnCount = o.getScanSize();

  this->rangeImage.setZero(rowCount, columnCount);
  this->intensityImage.setZero(rowCount, columnCount);
  this->organizedPoints.resize(rowCount, columnCount);
  this->rangeOtherLayers.clear();
  this->rangeResolution = 0.01;
  this->azimuthSpan = o.aperture * (o.rightToLeft ? +1.0 : -1.0);
  this->startAzimuth = o.aperture * (o.rightToLeft ? -0.5 : +0.5);

  double a = startAzimuth;
  const double Aa = azimuthSpan / o.getScanSize();

  for (size_t i = 0; i < o.getScanSize(); i++, a += Aa)
  {
    uint16_t& range_out = rangeImage(0, i);
    uint8_t& intensity_out = intensityImage(0, i);
    range_out = 0;
    intensity_out = 0;

    // Convert range into discrete units:
    const float r = o.getScanRange(i);
    const uint16_t r_discr = static_cast<uint16_t>((r / d2f(rangeResolution)) + 0.5f);

    if (!o.getScanRangeValidity(i) || r <= 0 || r >= o.maxRange) continue;

    range_out = r_discr;

    if (o.hasIntensity()) intensity_out = o.getScanIntensity(i);

    if (r > 0)
    {
      const auto ptLocal = mrpt::math::TPoint3Df(cos(a) * range_out, sin(a) * range_out, 0);

      organizedPoints(0, i) = sensorPose.composePoint(ptLocal);
    }
  }

  this->lidarModel = std::string("2D_SCAN_") + this->sensorLabel;

  MRPT_END
}

bool RotScan::fromGeneric(const mrpt::obs::CObservation& o)
{
  MRPT_START

  if (auto oVel = dynamic_cast<const CObservationVelodyneScan*>(&o); oVel)
  {
    fromVelodyne(*oVel);
    return true;
  }
  if (auto o2D = dynamic_cast<const CObservation2DRangeScan*>(&o); o2D)
  {
    fromScan2D(*o2D);
    return true;
  }
  return false;

  MRPT_END
}

void RotScan::load_impl() const
{
  // Already loaded?
  if (!isExternallyStored() || (isExternallyStored() && !rangeImage.empty())) return;

  const auto abs_filename = mrpt::io::lazy_load_absolute_path(m_external_file);
  ASSERT_FILE_EXISTS_(abs_filename);

  auto& me = const_cast<RotScan&>(*this);

  switch (m_externally_stored)
  {
    case ExternalStorageFormat::None:
      break;
    case ExternalStorageFormat::PlainTextFile:
    {
      me.loadFromTextFile(abs_filename);
    }
    break;
    case ExternalStorageFormat::MRPT_Serialization:
    {
      mrpt::io::CFileGZInputStream f(abs_filename);
      auto ar = mrpt::serialization::archiveFrom(f);
      readMatricesFrom(me, ar);
    }
    break;
  };
}

void RotScan::unload() const
{
  MRPT_START
  if (!isExternallyStored() || organizedPoints.empty()) return;

  // Free memory, saving to the file if it doesn't exist:
  const auto abs_filename = mrpt::io::lazy_load_absolute_path(m_external_file);

  if (!mrpt::system::fileExists(abs_filename))
  {
    switch (m_externally_stored)
    {
      case ExternalStorageFormat::None:
        break;
      case ExternalStorageFormat::PlainTextFile:
      {
        this->saveToTextFile(abs_filename);
      }
      break;
      case ExternalStorageFormat::MRPT_Serialization:
      {
        mrpt::io::CFileGZOutputStream f(abs_filename);
        auto ar = mrpt::serialization::archiveFrom(f);
        writeMatricesTo(*this, ar);
      }
      break;
    };
  }

  // Now we can safely free the mem:
  auto& me = const_cast<RotScan&>(*this);

  me.organizedPoints.resize(0, 0);
  me.rangeImage.resize(0, 0);
  me.intensityImage.resize(0, 0);
  me.rangeOtherLayers.clear();

  MRPT_END
}

void RotScan::setAsExternalStorage(const std::string& fileName, const ExternalStorageFormat fmt)
{
  MRPT_START
  ASSERTMSG_(!isExternallyStored(), "Already marked as externally-stored.");
  m_external_file = fileName;
  m_externally_stored = fmt;

  MRPT_END
}

// Write scan data to a plain text, each line has:
// `x y z intensity range row_idx col_idx`
bool RotScan::saveToTextFile(const std::string& filename) const
{
  ASSERT_(!organizedPoints.empty());
  ASSERT_EQUAL_(organizedPoints.size(), rangeImage.size());
  if (!intensityImage.empty())
  {
    ASSERT_EQUAL_(organizedPoints.size(), intensityImage.size());
  }

  std::ofstream f(filename);
  if (!f.is_open()) return false;

  for (size_t r = 0; r < rowCount; r++)
  {
    for (size_t c = 0; c < columnCount; c++)
    {
      const auto& pt = organizedPoints(r, c);

      const int valInt = intensityImage.empty() ? 0 : static_cast<int>(intensityImage(r, c));

      f << mrpt::format(
          "%g %g %g %f %i %zu %zu\n", pt.x, pt.y, pt.z, rangeResolution * rangeImage(r, c), valInt,
          r, c);
    }
  }
  return true;
}

bool RotScan::loadFromTextFile(const std::string& filename)
{
  mrpt::math::CMatrixFloat data;
  data.loadFromTextFile(filename);
  if (data.rows() == 0)
    THROW_EXCEPTION_FMT("Empty point cloud plain text file? `%s`", filename.c_str());

  //  `x y z range intensity row_idx col_idx`
  ASSERT_EQUAL_(data.cols(), 7UL);

  ASSERT_GT_(rowCount, 0);
  ASSERT_GT_(columnCount, 0);

  organizedPoints.resize(rowCount, columnCount);
  rangeImage.resize(rowCount, columnCount);
  intensityImage.resize(rowCount, columnCount);

  for (int i = 0; i < data.rows(); i++)
  {
    const size_t r = static_cast<size_t>(data(i, 5));
    const size_t c = static_cast<size_t>(data(i, 6));

    organizedPoints(r, c) = {data(i, 0), data(i, 1), data(i, 2)};
    rangeImage(r, c) = data(i, 3);
    intensityImage(r, c) = data(i, 4);
  }

  return true;
}
