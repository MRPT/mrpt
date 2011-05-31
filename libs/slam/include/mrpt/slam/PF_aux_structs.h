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
