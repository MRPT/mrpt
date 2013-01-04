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
#ifndef PF_aux_structs_H
#define PF_aux_structs_H

#include <mrpt/utils/utils_defs.h>
#include <vector>
#include <iostream>
#include <iterator>

#include <mrpt/slam/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		namespace detail
		{
			using namespace mrpt;
			using namespace mrpt::math;
			using namespace std;

			/** Auxiliary structure used in KLD-sampling in particle filters \sa CPosePDFParticles, CMultiMetricMapPDF */
			struct SLAM_IMPEXP TPoseBin2D
			{
				TPoseBin2D() : x(0),y(0),phi(0) { }

				int	x,y,phi; //!< Bin indices

				/** less-than ordering of bins for usage in STL containers */
				struct SLAM_IMPEXP lt_operator
				{
					inline bool operator()(const TPoseBin2D& s1, const TPoseBin2D& s2) const
					{
						if (s1.x < s2.x)  return true;
						if (s1.x > s2.x)  return false;
						if (s1.y < s2.y)  return true;
						if (s1.y > s2.y)  return false;
						return s1.phi<s2.phi;
					}
				};
			};

			/** Auxiliary structure   */
			struct SLAM_IMPEXP TPathBin2D
			{
				std::vector<TPoseBin2D> bins;

				/** less-than ordering of bins for usage in STL containers */
				struct SLAM_IMPEXP lt_operator
				{
					bool operator()(const TPathBin2D& s1, const TPathBin2D& s2) const
					{
						ASSERT_(s1.bins.size()==s2.bins.size())
						for (size_t i=0;i<s1.bins.size();i++)
						{
							if (s1.bins[i].x < s2.bins[i].x)  return true;
							if (s1.bins[i].x > s2.bins[i].x)  return false;
							if (s1.bins[i].y < s2.bins[i].y)  return true;
							if (s1.bins[i].y > s2.bins[i].y)  return false;
							if (s1.bins[i].phi < s2.bins[i].phi)  return true;
							if (s1.bins[i].phi > s2.bins[i].phi)  return false;
							// else, keep comparing:
						}
						return false; // If they're exactly equal, s1 is NOT < s2.
					}
				};
			};

			/** Auxiliary structure used in KLD-sampling in particle filters \sa CPosePDFParticles, CMultiMetricMapPDF */
			struct SLAM_IMPEXP TPoseBin3D
			{
				TPoseBin3D() : x(0),y(0),z(0),yaw(0),pitch(0),roll(0) { }

				int	x,y,z,yaw,pitch,roll; //!< Bin indices

				/** less-than ordering of bins for usage in STL containers */
				struct SLAM_IMPEXP lt_operator
				{
					bool operator()(const TPoseBin3D& s1, const TPoseBin3D& s2) const
					{
						if (s1.x < s2.x)  return true;
						if (s1.x > s2.x)  return false;
						if (s1.y < s2.y)  return true;
						if (s1.y > s2.y)  return false;
						if (s1.z < s2.z)  return true;
						if (s1.z > s2.z)  return false;
						if (s1.yaw < s2.yaw)  return true;
						if (s1.yaw > s2.yaw)  return false;
						if (s1.pitch < s2.pitch)  return true;
						if (s1.pitch > s2.pitch)  return false;
						return s1.roll<s2.roll;
					}
				};
			};


		} // End of namespace
	} // End of namespace
} // End of namespace

#endif
