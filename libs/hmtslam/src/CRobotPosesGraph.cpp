/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "hmtslam-precomp.h" // Precomp header

#include <mrpt/hmtslam/CRobotPosesGraph.h>

using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::poses;
using namespace mrpt::hmtslam;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CRobotPosesGraph, CSerializable , mrpt::hmtslam)


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CRobotPosesGraph::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		uint32_t   N = static_cast<uint32_t>(size());
		out << N;

		for (std::map<TPoseID,TPoseInfo>::const_iterator it=begin();it!=end();++it)
		{
			out << it->first
			    << it->second.sf
			    << it->second.pdf;
		}
	}
}

/*---------------------------------------------------------------
						readFromStream
  ---------------------------------------------------------------*/
void  CRobotPosesGraph::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			uint32_t   i,N;
			in >> N;
			clear();

			for (i=0;i<N;i++)
			{
				TPoseID  poseid;
				in >> poseid;

				TPoseInfo  &info = (*this)[poseid];

				in >> info.sf
				   >> info.pdf;
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
					insertIntoMetricMap
  ---------------------------------------------------------------*/
void CRobotPosesGraph::insertIntoMetricMap( CMultiMetricMap	&metricMap ) const
{
	CPose3D  meanPose;
	for (std::map<TPoseID,TPoseInfo>::const_iterator it=begin();it!=end();++it)
	{
		it->second.pdf.getMean(meanPose);
		it->second.sf.insertObservationsInto( &metricMap, &meanPose );
	}
}

/*---------------------------------------------------------------
					convertIntoSimplemap
  ---------------------------------------------------------------*/
void CRobotPosesGraph::convertIntoSimplemap( CSimpleMap &out_simplemap) const
{
	out_simplemap.clear();
	for (std::map<TPoseID,TPoseInfo>::const_iterator it=begin();it!=end();++it)
		out_simplemap.insert( &it->second.pdf, it->second.sf );
}
