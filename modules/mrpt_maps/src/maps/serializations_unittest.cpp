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

#define MRPT_NO_WARN_BIG_HDR  // Yes, we really want to include all classes.
#include <CTraitsTest.h>
#include <gtest/gtest.h>
#include <mrpt/io/CMemoryStream.h>
#include <mrpt/maps.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CObservationRotatingScan.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/viz/CAngularObservationMesh.h>
#include <mrpt/viz/CPlanarLaserScan.h>

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::viz;
using namespace mrpt::obs;
using namespace mrpt::io;
using namespace mrpt::serialization;
using namespace std;

template class mrpt::CTraitsTest<CBeacon>;
template class mrpt::CTraitsTest<CBeaconMap>;
template class mrpt::CTraitsTest<CGasConcentrationGridMap2D>;
template class mrpt::CTraitsTest<CWirelessPowerGridMap2D>;
template class mrpt::CTraitsTest<CHeightGridMap2D>;
template class mrpt::CTraitsTest<CReflectivityGridMap2D>;
template class mrpt::CTraitsTest<COccupancyGridMap2D>;
template class mrpt::CTraitsTest<COccupancyGridMap3D>;
template class mrpt::CTraitsTest<CSimplePointsMap>;
template class mrpt::CTraitsTest<CGenericPointsMap>;
template class mrpt::CTraitsTest<CRandomFieldGridMap3D>;
template class mrpt::CTraitsTest<COctoMap>;
template class mrpt::CTraitsTest<CColouredOctoMap>;
template class mrpt::CTraitsTest<CVoxelMap>;
template class mrpt::CTraitsTest<CVoxelMapRGB>;
template class mrpt::CTraitsTest<CSinCosLookUpTableFor2DScans>;
// obs:
template class mrpt::CTraitsTest<CObservationPointCloud>;
template class mrpt::CTraitsTest<CObservationRotatingScan>;
// opengl:
template class mrpt::CTraitsTest<CAngularObservationMesh>;
template class mrpt::CTraitsTest<CPlanarLaserScan>;

// Create a set of classes, then serialize and deserialize to test possible
// bugs:
TEST(SerializeTestMaps, WriteReadToMem)
{
  const mrpt::rtti::TRuntimeClassId* lstClasses[] = {
      CLASS_ID(CBeacon),
      CLASS_ID(CBeaconMap),
      CLASS_ID(CGasConcentrationGridMap2D),
      CLASS_ID(CWirelessPowerGridMap2D),
      CLASS_ID(CHeightGridMap2D),
      CLASS_ID(CReflectivityGridMap2D),
      CLASS_ID(COccupancyGridMap2D),
      CLASS_ID(COccupancyGridMap3D),
      CLASS_ID(CSimplePointsMap),
      CLASS_ID(CRandomFieldGridMap3D),
      CLASS_ID(CGenericPointsMap),
      CLASS_ID(COctoMap),
      CLASS_ID(CColouredOctoMap),
      CLASS_ID(CVoxelMap),
      CLASS_ID(CVoxelMapRGB),
      // obs:
      CLASS_ID(CObservationPointCloud),
      CLASS_ID(CObservationRotatingScan),
      // opengl:
      CLASS_ID(CAngularObservationMesh),
      CLASS_ID(CPlanarLaserScan),
  };

  for (auto& classInfo : lstClasses)
  {
    try
    {
      CMemoryStream buf;
      auto arch = mrpt::serialization::archiveFrom(buf);
      std::cout << "Serializing " << classInfo->className << "...";
      {
        auto o = mrpt::ptr_cast<CSerializable>::from(classInfo->createObject());
        arch << *o;
        o.reset();
      }
      std::cout << "OK.\n";

      std::cout << "  Deserializing it...";

      CSerializable::Ptr recons;
      buf.Seek(0);
      arch >> recons;

      std::cout << "OK.\n";
    }
    catch (const std::exception& e)
    {
      GTEST_FAIL() << "Exception during serialization test for class '" << classInfo->className
                   << "':\n"
                   << e.what() << "\n";
    }
  }
}
