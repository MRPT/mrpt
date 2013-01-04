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

#include <mrpt/maps.h>  // Precompiled header

#include <mrpt/slam/COctoMap.h>
#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/utils/color_maps.h>

#include <octomap/octomap.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::math;

#define OCTOMAP_PTR        static_cast<octomap::OcTree*>(m_octomap)
#define OCTOMAP_PTR_CONST  static_cast<const octomap::OcTree*>(m_octomap)

#define PARENT_OCTOMAP_PTR        static_cast<octomap::OcTree*>(m_parent->m_octomap)
#define PARENT_OCTOMAP_PTR_CONST  static_cast<const octomap::OcTree*>(m_parent->m_octomap)


MRPT_TODO("TRenderingOptions should be serialized and loaded from .ini files in multimetricmaps")

COctoMap::TRenderingOptions::TRenderingOptions() :
	generateGridLines      (false),
	generateOccupiedVoxels (true),
	visibleOccupiedVoxels  (true),
	generateFreeVoxels     (true),
	visibleFreeVoxels      (true)
{
}

/** Returns a 3D object representing the map.
	*/
void COctoMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr &outObj ) const
{
	mrpt::opengl::COctoMapVoxelsPtr gl_obj = mrpt::opengl::COctoMapVoxels::Create();
	this->getAsOctoMapVoxels(*gl_obj);
	outObj->insert(gl_obj);
}

/** Builds a renderizable representation of the octomap as a mrpt::opengl::COctoMapVoxels object. */
void COctoMap::getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const
{
	// Go thru all voxels:
	octomap::OcTree *tree = const_cast<octomap::OcTree*>(OCTOMAP_PTR_CONST); // because there's no a "OcTree::const_iterator"

	//OcTreeVolume voxel; // current voxel, possibly transformed 
	octomap::OcTree::tree_iterator it_end = tree->end_tree();

	const unsigned char max_depth = 0; // all

	gl_obj.clear();
	gl_obj.reserveGridCubes( this->calcNumNodes() );

	gl_obj.resizeVoxelSets(2); // 2 sets of voxels: occupied & free

	gl_obj.showVoxels(VOXEL_SET_OCCUPIED,  renderingOptions.visibleOccupiedVoxels );
	gl_obj.showVoxels(VOXEL_SET_FREESPACE, renderingOptions.visibleFreeVoxels );
		
	const size_t nLeafs = this->getNumLeafNodes();
	gl_obj.reserveVoxels(VOXEL_SET_OCCUPIED, nLeafs);
	gl_obj.reserveVoxels(VOXEL_SET_FREESPACE, nLeafs);
	
	for(octomap::OcTree::tree_iterator it = tree->begin_tree(max_depth);it!=it_end; ++it) 
	{
		const octomap::point3d vx_center = it.getCoordinate();
		const double           vx_length = it.getSize();
		const double           L = 0.5*vx_length;

		if (it.isLeaf()) 
		{
			// voxels for leaf nodes
			const double occ = it->getOccupancy();
			if ( (occ>=0.5 && renderingOptions.generateOccupiedVoxels) ||
				 (occ<0.5  && renderingOptions.generateFreeVoxels) )
			{
				const double c = 1-occ; // 0=max. occupancy
				mrpt::utils::TColor vx_color(c*255,c*255,c*255);

				const size_t vx_set = (tree->isNodeOccupied(*it)) ? VOXEL_SET_OCCUPIED:VOXEL_SET_FREESPACE;

				gl_obj.push_back_Voxel(vx_set,COctoMapVoxels::TVoxel( TPoint3D(vx_center.x(),vx_center.y(),vx_center.z()) ,vx_length, vx_color) );				
			}
		}
		else if (renderingOptions.generateGridLines)
		{
			// Not leaf-nodes:
			const mrpt::math::TPoint3D pt_min ( vx_center.x() - L, vx_center.y() - L, vx_center.z() - L);
			const mrpt::math::TPoint3D pt_max ( vx_center.x() + L, vx_center.y() + L, vx_center.z() + L);
			gl_obj.push_back_GridCube( COctoMapVoxels::TGridCube( pt_min, pt_max ) );
		}
	} // end for each voxel

	// Set bounding box:
	{
		mrpt::math::TPoint3D bbmin, bbmax;
		this->getMetricMin(bbmin.x,bbmin.y,bbmin.z);
		this->getMetricMax(bbmax.x,bbmax.y,bbmax.z);
		gl_obj.setBoundingBox(bbmin, bbmax);
	}
	
}
