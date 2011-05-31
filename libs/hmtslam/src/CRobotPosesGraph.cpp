/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                       http://www.mrpt.org/                                |
   |                                                                           |
   |   Copyright (C) 2005-2011  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/hmtslam.h> // Precomp header

using namespace mrpt::slam;
using namespace mrpt::hmtslam;
using namespace std;

IMPLEMENTS_SERIALIZABLE( CRobotPosesGraph, CSerializable , mrpt::hmtslam)


/*---------------------------------------------------------------
						writeToStream
  ---------------------------------------------------------------*/
void  CRobotPosesGraph::writeToStream(CStream &out,int *version) const
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
void  CRobotPosesGraph::readFromStream(CStream &in,int version)
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
