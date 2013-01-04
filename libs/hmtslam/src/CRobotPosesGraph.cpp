/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
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
