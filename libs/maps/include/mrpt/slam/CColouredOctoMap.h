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

#ifndef MRPT_CColouredOctoMap_H
#define MRPT_CColouredOctoMap_H

#include <mrpt/slam/COctoMapBase.h>
#include <mrpt/otherlibs/octomap/octomap.h>
#include <mrpt/otherlibs/octomap/ColorOcTree.h>

#include <mrpt/maps/link_pragmas.h>

namespace mrpt
{
	namespace slam
	{
		class CPointsMap;
		class CObservation2DRangeScan;
		class CObservation3DRangeScan;

		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CColouredOctoMap , CMetricMap, MAPS_IMPEXP )

		/** A three-dimensional probabilistic occupancy grid, implemented as an octo-tree with the "octomap" C++ library.
		 *  This version stores both, occupancy information and RGB colour data at each octree node. See the base class mrpt::slam::COctoMapBase.
		 *
		 * \sa CMetricMap, the example in "MRPT/samples/octomap_simple"
	  	 * \ingroup mrpt_maps_grp
		 */
		class MAPS_IMPEXP CColouredOctoMap : public COctoMapBase<octomap::ColorOcTree,octomap::ColorOcTreeNode>
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CColouredOctoMap )

		 public:
			 CColouredOctoMap(const double resolution=0.10);          //!< Default constructor
			 virtual ~CColouredOctoMap(); //!< Destructor

			/** This allows the user to select the desired method to update voxels colour.
				SET = Set the colour of the voxel at (x,y,z) directly
				AVERAGE = Set the colour of the voxel at (x,y,z) as the mean of its previous colour and the new observed one.
				INTEGRATE = Calculate the new colour of the voxel at (x,y,z) using this formula: prev_color*node_prob +  new_color*(0.99-node_prob)
				If there isn't any previous color, any method is equivalent to SET.
				INTEGRATE is the default option*/
			enum TColourUpdate
			{
				INTEGRATE = 0,
				SET,
				AVERAGE
			};

			/** Get the RGB colour of a point
				* \return false if the point is not mapped, in which case the returned colour is undefined. */
			bool getPointColour(const float x, const float y, const float z, char& r, char& g, char& b) const;

			/** Manually update the colour of the voxel at (x,y,z) */
			void updateVoxelColour(const double x, const double y, const double z, const char r, const char g, const char b);

			///Set the method used to update voxels colour
			void setVoxelColourMethod(TColourUpdate new_method) {m_colour_method = new_method;}

			///Get the method used to update voxels colour
			TColourUpdate getVoxelColourMethod() {return m_colour_method;}

			virtual void getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const;

		protected:

			bool internal_insertObservation(const CObservation *obs,const CPose3D *robotPose);

			TColourUpdate m_colour_method;		//!Method used to updated voxels colour.

		}; // End of class def.
	} // End of namespace

} // End of namespace

#endif
