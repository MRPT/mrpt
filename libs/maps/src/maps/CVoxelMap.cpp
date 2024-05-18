/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header
//
#include <mrpt/maps/../obs/CObservationPointCloud.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/maps/CVoxelMap.h>

#include <mrpt/maps/bonxai/serialization.hpp>

using namespace mrpt::maps;
using namespace std::string_literals;  // "..."s

//  =========== Begin of Map definition ============
MAP_DEFINITION_REGISTER("mrpt::maps::CVoxelMap,voxelMap", mrpt::maps::CVoxelMap)

CVoxelMap::TMapDefinition::TMapDefinition() = default;
void CVoxelMap::TMapDefinition::loadFromConfigFile_map_specific(
    const mrpt::config::CConfigFileBase& source, const std::string& sectionNamePrefix)
{
  // [<sectionNamePrefix>+"_creationOpts"]
  const std::string sSectCreation = sectionNamePrefix + "_creationOpts"s;
  MRPT_LOAD_CONFIG_VAR(resolution, double, source, sSectCreation);

  insertionOpts.loadFromConfigFile(source, sectionNamePrefix + "_insertOpts"s);
  likelihoodOpts.loadFromConfigFile(source, sectionNamePrefix + "_likelihoodOpts"s);
}

void CVoxelMap::TMapDefinition::dumpToTextStream_map_specific(std::ostream& out) const
{
  LOADABLEOPTS_DUMP_VAR(resolution, double);

  this->insertionOpts.dumpToTextStream(out);
  this->likelihoodOpts.dumpToTextStream(out);
}

mrpt::maps::CMetricMap::Ptr CVoxelMap::internal_CreateFromMapDefinition(
    const mrpt::maps::TMetricMapInitializer& _def)
{
  const CVoxelMap::TMapDefinition& def = *dynamic_cast<const CVoxelMap::TMapDefinition*>(&_def);
  auto obj = CVoxelMap::Create(def.resolution);
  obj->insertionOptions = def.insertionOpts;
  obj->likelihoodOptions = def.likelihoodOpts;
  return obj;
}
//  =========== End of Map definition Block =========

IMPLEMENTS_SERIALIZABLE(CVoxelMap, CMetricMap, mrpt::maps)

/*---------------------------------------------------------------
            Constructor
  ---------------------------------------------------------------*/
CVoxelMap::~CVoxelMap() = default;

uint8_t CVoxelMap::serializeGetVersion() const { return 0; }
void CVoxelMap::serializeTo(mrpt::serialization::CArchive& out) const
{
  insertionOptions.writeToStream(out);
  likelihoodOptions.writeToStream(out);
  renderingOptions.writeToStream(out);  // Added in v1
  out << genericMapParams;

  // grid data:
  std::stringstream ss;
  Bonxai::Serialize(ss, grid());
  out << ss.str();
}

void CVoxelMap::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      insertionOptions.readFromStream(in);
      likelihoodOptions.readFromStream(in);
      renderingOptions.readFromStream(in);
      in >> genericMapParams;

      this->clear();

      // grid data:
      std::string msg;
      in >> msg;
      std::istringstream ifile(msg, std::ios::binary);

      char header[256];
      ifile.getline(header, 256);
      Bonxai::HeaderInfo info = Bonxai::GetHeaderInfo(header);

      m_impl = std::make_unique<Impl>(Bonxai::Deserialize<voxel_node_t>(ifile, info));
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  };
}

bool CVoxelMap::internal_insertObservation_Pts(
    const mrpt::obs::CObservationPointCloud& obs,
    const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  if (!obs.pointcloud || obs.pointcloud->empty()) return false;

  mrpt::math::TPoint3D sensorPt;
  mrpt::poses::CPose3D localSensorPose, globalSensorPose;
  obs.getSensorPose(localSensorPose);
  if (robotPose)  //
    globalSensorPose = *robotPose + localSensorPose;
  else
    globalSensorPose = localSensorPose;

  sensorPt = globalSensorPose.translation();

  // Remove distant voxel option?
  if (insertionOptions.remove_voxels_farther_than > 0)
  {
    const int distInGrid = static_cast<int>(
        std::ceil(insertionOptions.remove_voxels_farther_than * m_impl->grid.inv_resolution));

    const auto idxCurObs = Bonxai::PosToCoord(
        {sensorPt.x, sensorPt.y, sensorPt.z}, base_t::m_impl->grid.inv_resolution);

    this->m_impl->grid.forEachCell(
        [&](voxel_node_t& v, const Bonxai::CoordT& c)
        {
          // manhattan distance:
          const int dist = mrpt::max3(
              std::abs(c.x - idxCurObs.x), std::abs(c.y - idxCurObs.y),
              std::abs(c.z - idxCurObs.z));
          if (dist < distInGrid) return;

          // delete:
          // Bonxai doesn't seem to have an erase()...
          v.occupancyRef() = {};  // reset to "unseen voxel"
        });
  }

  // Insert rays:
  if (insertionOptions.ray_trace_free_space)
    insertPointCloudAsRays(*obs.pointcloud, sensorPt, globalSensorPose);
  else
    insertPointCloudAsEndPoints(*obs.pointcloud, sensorPt, globalSensorPose);

  return true;
}

bool CVoxelMap::internal_insertObservation(
    const mrpt::obs::CObservation& obs, const std::optional<const mrpt::poses::CPose3D>& robotPose)
{
  if (auto obsPts = dynamic_cast<const mrpt::obs::CObservationPointCloud*>(&obs); obsPts)
  {
    return internal_insertObservation_Pts(*obsPts, robotPose);
  }

  // Auxiliary 3D point cloud:
  mrpt::maps::CSimplePointsMap pts;
  pts.insertObservation(obs, robotPose);

  if (pts.empty()) return false;

  mrpt::math::TPoint3D sensorPt;
  mrpt::poses::CPose3D localSensorPose;
  obs.getSensorPose(localSensorPose);
  if (robotPose)
  {
    // compose:
    sensorPt = (*robotPose + localSensorPose).translation();
  }
  else
  {
    sensorPt = localSensorPose.translation();
  }

  // Insert rays:
  if (insertionOptions.ray_trace_free_space)
    insertPointCloudAsRays(pts, sensorPt);
  else
    insertPointCloudAsEndPoints(pts, sensorPt);
  return true;
}

double CVoxelMap::internal_computeObservationLikelihood(
    const mrpt::obs::CObservation& obs, const mrpt::poses::CPose3D& takenFrom) const
{
  // build aux 3D pointcloud:
  mrpt::maps::CSimplePointsMap pts;
  pts.insertObservation(obs, takenFrom);

  if (pts.empty()) return 0;

  double log_lik = .0;  // cummulative log likelihoo

  auto lambdaPointLikelihood = [&](float x, float y, float z)
  {
    double probOcc = 0;
    const bool voxelExists = getPointOccupancy(x, y, z, probOcc);
    if (!voxelExists) return;
    log_lik += probOcc;
  };

  const auto& xs = pts.getPointsBufferRef_x();
  const auto& ys = pts.getPointsBufferRef_y();
  const auto& zs = pts.getPointsBufferRef_z();

  if (pts.size() <= likelihoodOptions.decimate_up_to)
  {
    for (size_t i = 0; i < pts.size(); ++i) lambdaPointLikelihood(xs[i], ys[i], zs[i]);
  }
  else
  {
    const double delta = static_cast<double>(pts.size()) / likelihoodOptions.decimate_up_to;

    for (size_t i = 0; i < likelihoodOptions.decimate_up_to; ++i)
    {
      const auto idx = static_cast<size_t>(i * delta);
      lambdaPointLikelihood(xs[idx], ys[idx], zs[idx]);
    }
  }

  return log_lik;
}
