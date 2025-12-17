/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2025, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/maps/CColouredPointsMap.h>
#include <mrpt/maps/CPointsMapXYZI.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/math/ops_containers.h>  // minimum_maximum()
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>

#include <fstream>
#include <iostream>

using namespace mrpt::obs;

IMPLEMENTS_SERIALIZABLE(CObservationPointCloud, CObservation, mrpt::obs);

CObservationPointCloud::CObservationPointCloud(const CObservation3DRangeScan& o)
{
  pointcloud = mrpt::maps::CSimplePointsMap::Create();
  pointcloud->loadFromRangeScan(o, std::nullopt);
}

void CObservationPointCloud::getSensorPose(mrpt::poses::CPose3D& out_sensorPose) const
{
  out_sensorPose = sensorPose;
}
void CObservationPointCloud::setSensorPose(const mrpt::poses::CPose3D& p) { sensorPose = p; }
void CObservationPointCloud::getDescriptionAsText(std::ostream& o) const
{
  CObservation::getDescriptionAsText(o);
  o << "Homogeneous matrix for the sensor pose wrt vehicle:\n";
  o << sensorPose.getHomogeneousMatrixVal<mrpt::math::CMatrixDouble44>() << "\n"
    << sensorPose << "\n";

  o << "Pointcloud class: ";
  if (!this->pointcloud)
  {
    o << "nullptr\n";
  }
  else
  {
    o << pointcloud->GetRuntimeClass()->className << "\n";
    o << "Number of points: " << pointcloud->size() << "\n";

    // Show channel stats:
    for (const auto& fieldName : pointcloud->getPointFieldNames_float())
    {
      const auto* data = pointcloud->getPointsBufferRef_float_field(fieldName);
      if (!data)
      {
        continue;
      }
      float Imin = 0, Imax = 0;
      if (!data->empty())
      {
        mrpt::math::minimum_maximum(*data, Imin, Imax);
      }
      o << mrpt::format(
          "Field: %-20.*s (float32): min=%7.02f max=%7.02f (%zu entries)\n",
          static_cast<int>(fieldName.size()), fieldName.data(), Imin, Imax, data->size());
    }
    for (const auto& fieldName : pointcloud->getPointFieldNames_uint16())
    {
      const auto* data = pointcloud->getPointsBufferRef_uint_field(fieldName);
      if (!data)
      {
        continue;
      }
      uint16_t Imin = 0, Imax = 0;
      if (!data->empty())
      {
        mrpt::math::minimum_maximum(*data, Imin, Imax);
      }
      o << mrpt::format(
          "Field: %-20.*s  (uint16): min=%7hu max=%7hu (%zu entries)\n",
          static_cast<int>(fieldName.size()), fieldName.data(), Imin, Imax, data->size());
    }
  }

  if (m_externally_stored != ExternalStorageFormat::None)
    o << "Pointcloud is stored externally in format `" << static_cast<int>(m_externally_stored)
      << "` in file `" << m_external_file << "`\n";
}

uint8_t CObservationPointCloud::serializeGetVersion() const { return 0; }
void CObservationPointCloud::serializeTo(mrpt::serialization::CArchive& out) const
{
  out << sensorLabel << timestamp;  // Base class data

  out << sensorPose;
  out.WriteAs<uint8_t>(m_externally_stored);

  if (isExternallyStored())
  {
    out << m_external_file;
  }
  else
  {
    out << pointcloud;
  }
}

void CObservationPointCloud::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      pointcloud.reset();
      in >> sensorLabel >> timestamp;  // Base class data

      in >> sensorPose;
      m_externally_stored = static_cast<ExternalStorageFormat>(in.ReadPOD<uint8_t>());

      if (isExternallyStored())
      {
        in >> m_external_file;
      }
      else
      {
        m_external_file.clear();
        in >> pointcloud;
      }
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

void CObservationPointCloud::load_impl() const
{
  MRPT_START

  const thread_local bool MRPT_DEBUG_OBSPTS_LAZY_LOAD =
      mrpt::get_env<bool>("MRPT_DEBUG_OBSPTS_LAZY_LOAD", false);
  if (MRPT_DEBUG_OBSPTS_LAZY_LOAD)
    std::cout << "[CObservationPointCloud::load()] Called on this="
              << reinterpret_cast<const void*>(this) << std::endl;

  // Already loaded?
  if (!isExternallyStored() || (isExternallyStored() && pointcloud)) return;

  const auto abs_filename = mrpt::io::lazy_load_absolute_path(m_external_file);
  ASSERT_FILE_EXISTS_(abs_filename);

  switch (m_externally_stored)
  {
    case ExternalStorageFormat::None:
      break;
    case ExternalStorageFormat::KittiBinFile:
    {
      auto pts = mrpt::maps::CPointsMapXYZI::Create();
      bool ok = pts->loadFromKittiVelodyneFile(abs_filename);
      ASSERTMSG_(
          ok, mrpt::format(
                  "[kitti format] Error loading lazy-load point cloud "
                  "file: '%s'",
                  abs_filename.c_str()));
      auto pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(pts);
      const_cast<mrpt::maps::CPointsMap::Ptr&>(pointcloud) = pc;
    }
    break;
    case ExternalStorageFormat::PlainTextFile:
    {
      mrpt::math::CMatrixFloat data;
      data.loadFromTextFile(abs_filename);
      if (data.rows() == 0)
        THROW_EXCEPTION_FMT(
            "Empty external point cloud plain text file? `%s`", abs_filename.c_str());

      mrpt::maps::CPointsMap::Ptr pc;

      if (data.cols() == 3 || data.cols() == 2)
      {
        pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
            mrpt::maps::CSimplePointsMap::Create());
      }
      else if (data.cols() == 6)
      {
        pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(
            mrpt::maps::CColouredPointsMap::Create());
      }
      else if (data.cols() == 4)
      {
        pc =
            std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(mrpt::maps::CPointsMapXYZI::Create());
      }
      else
        THROW_EXCEPTION_FMT(
            "Unexpected number of columns in point cloud file: cols=%u",
            static_cast<unsigned int>(data.cols()));

      pc->resize(data.rows());
      std::vector<float> vals(data.cols());
      for (int i = 0; i < data.rows(); i++)
      {
        data.extractRow(i, vals);
        pc->setPointAllFieldsFast(i, vals);
      }

      // Assign:
      const_cast<mrpt::maps::CPointsMap::Ptr&>(pointcloud) = pc;
    }
    break;
    case ExternalStorageFormat::MRPT_Serialization:
    {
      mrpt::io::CFileGZInputStream f(abs_filename);
      auto ar = mrpt::serialization::archiveFrom(f);
      auto obj = ar.ReadObject();
      auto pc = std::dynamic_pointer_cast<mrpt::maps::CPointsMap>(obj);
      const_cast<mrpt::maps::CPointsMap::Ptr&>(pointcloud) = pc;

      ASSERTMSG_(
          pointcloud, mrpt::format(
                          "[mrpt-serialization format] Error loading "
                          "lazy-load point cloud file: %s",
                          abs_filename.c_str()));
    }
    break;
  };

  MRPT_END
}
void CObservationPointCloud::unload() const
{
  MRPT_START

  const thread_local bool MRPT_DEBUG_OBSPTS_LAZY_LOAD =
      mrpt::get_env<bool>("MRPT_DEBUG_OBSPTS_LAZY_LOAD", false);
  if (MRPT_DEBUG_OBSPTS_LAZY_LOAD)
    std::cout << "[CObservationPointCloud::unload()] Called on this="
              << reinterpret_cast<const void*>(this) << std::endl;

  if (!isExternallyStored() || !pointcloud) return;

  // Free memory, saving to the file if it doesn't exist:
  const auto abs_filename = mrpt::io::lazy_load_absolute_path(m_external_file);

  if (!mrpt::system::fileExists(abs_filename))
  {
    switch (m_externally_stored)
    {
      case ExternalStorageFormat::None:
        break;
      case ExternalStorageFormat::KittiBinFile:
      {
        THROW_EXCEPTION("Saving to kitti format not supported.");
      }
      case ExternalStorageFormat::PlainTextFile:
      {
        std::ofstream f(abs_filename);
        ASSERT_(f.is_open());
        std::vector<float> row;
        for (size_t i = 0; i < pointcloud->size(); i++)
        {
          pointcloud->getPointAllFieldsFast(i, row);
          for (const float v : row) f << v << " ";
          f << "\n";
        }
      }
      break;
      case ExternalStorageFormat::MRPT_Serialization:
      {
        mrpt::io::CFileGZOutputStream f(abs_filename);
        auto ar = mrpt::serialization::archiveFrom(f);
        ar << *pointcloud;
      }
      break;
    };
  }

  // Now we can safely free the mem:
  auto& me = const_cast<CObservationPointCloud&>(*this);
  me.pointcloud.reset();

  MRPT_END
}

void CObservationPointCloud::setAsExternalStorage(
    const std::string& fileName, const CObservationPointCloud::ExternalStorageFormat fmt)
{
  MRPT_START
  ASSERTMSG_(!isExternallyStored(), "Already marked as externally-stored.");
  m_external_file = fileName;
  m_externally_stored = fmt;

  MRPT_END
}
