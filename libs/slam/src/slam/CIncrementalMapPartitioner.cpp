/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/slam/CIncrementalMapPartitioner.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/slam/observations_overlap.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/graphs/CGraphPartitioner.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/serialization/stl_serialization.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfObjects.h>
#include <mrpt/opengl/CSphere.h>
#include <mrpt/opengl/CSimpleLine.h>

using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::graphs;
using namespace mrpt::poses;
using namespace mrpt;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CIncrementalMapPartitioner, CSerializable, mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CIncrementalMapPartitioner::CIncrementalMapPartitioner()
	: COutputLogger("CIncrementalMapPartitioner"),
	  options(),
	  m_individualFrames(),
	  m_individualMaps(),
	  m_A(0, 0),
	  m_last_partition(),
	  m_last_last_partition_are_new_ones(false),
	  m_modified_nodes()
{
	clear();
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CIncrementalMapPartitioner::~CIncrementalMapPartitioner() { clear(); }
/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CIncrementalMapPartitioner::TOptions::TOptions()
	: partitionThreshold(1.0f),
	  gridResolution(0.10f),
	  minDistForCorrespondence(0.20f),
	  minMahaDistForCorrespondence(2.0f),
	  forceBisectionOnly(false),
	  useMapMatching(true),
	  minimumNumberElementsEachCluster(1)
{
}

/*---------------------------------------------------------------
						loadFromConfigFile
  ---------------------------------------------------------------*/
void CIncrementalMapPartitioner::TOptions::loadFromConfigFile(
	const mrpt::config::CConfigFileBase& source, const string& section)
{
	MRPT_START

	MRPT_LOAD_CONFIG_VAR(partitionThreshold, float, source, section);
	MRPT_LOAD_CONFIG_VAR(gridResolution, float, source, section);
	MRPT_LOAD_CONFIG_VAR(minDistForCorrespondence, float, source, section);
	MRPT_LOAD_CONFIG_VAR(forceBisectionOnly, bool, source, section);
	MRPT_LOAD_CONFIG_VAR(useMapMatching, bool, source, section);
	MRPT_LOAD_CONFIG_VAR(
		minimumNumberElementsEachCluster, int, source, section);

	MRPT_END
}

/*---------------------------------------------------------------
						dumpToTextStream
  ---------------------------------------------------------------*/
void CIncrementalMapPartitioner::TOptions::dumpToTextStreamstd::ostream& out, int* version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_individualFrames << m_individualMaps << m_A << m_last_partition
			<< m_last_last_partition_are_new_ones << m_modified_nodes;
	}
}

/*---------------------------------------------------------------
					addMapFrame
  ---------------------------------------------------------------*/
unsigned int CIncrementalMapPartitioner::addMapFrame(
	const CSensoryFrame& frame, const CPose3DPDF& robotPose3D)
{
	return addMapFrame(
		CSensoryFrame::Ptr(new CSensoryFrame(frame)),
		std::dynamic_pointer_cast<CPose3DPDF>(
			robotPose3D.duplicateGetSmartPtr()));
}
