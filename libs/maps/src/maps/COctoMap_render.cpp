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

#include <mrpt/otherlibs/octomap/octomap.h>

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


COctoMap::TRenderingOptions::TRenderingOptions() :
	generateGridLines      (false),
	generateOccupiedVoxels (true),
	visibleOccupiedVoxels  (true),
	generateFreeVoxels     (true),
	visibleFreeVoxels      (true)
{
}

void COctoMap::TRenderingOptions::writeToStream(CStream &out) const
{
	const int8_t version = 0;
	out << version;

	out << 	generateGridLines << generateOccupiedVoxels << visibleOccupiedVoxels
	    << generateFreeVoxels << visibleFreeVoxels;
;
}

void COctoMap::TRenderingOptions::readFromStream(CStream &in)
{
	int8_t version;
	in >> version;
	switch(version)
	{
		case 0:
		{
			in >> generateGridLines >> generateOccupiedVoxels >> visibleOccupiedVoxels
				>> generateFreeVoxels >> visibleFreeVoxels;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	}
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
	const TColorf general_color = gl_obj.getColor();
	const TColor general_color_u(general_color.R*255,general_color.G*255,general_color.B*255,general_color.A*255);

	gl_obj.clear();
	gl_obj.reserveGridCubes( this->calcNumNodes() );

	gl_obj.resizeVoxelSets(2); // 2 sets of voxels: occupied & free

	gl_obj.showVoxels(VOXEL_SET_OCCUPIED,  renderingOptions.visibleOccupiedVoxels );
	gl_obj.showVoxels(VOXEL_SET_FREESPACE, renderingOptions.visibleFreeVoxels );

	const size_t nLeafs = this->getNumLeafNodes();
	gl_obj.reserveVoxels(VOXEL_SET_OCCUPIED, nLeafs);
	gl_obj.reserveVoxels(VOXEL_SET_FREESPACE, nLeafs);

	double xmin, xmax, ymin, ymax, zmin, zmax, inv_dz;
	this->getMetricMin(xmin, ymin, zmin);
	this->getMetricMax(xmax, ymax, zmax);
	inv_dz = 1/(zmax-zmin + 0.01);

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
				mrpt::utils::TColor vx_color;
				double coefc, coeft;
				switch (gl_obj.getVisualizationMode()) {
				case COctoMapVoxels::FIXED:
					vx_color = general_color_u;
					break;
				case COctoMapVoxels::COLOR_FROM_HEIGHT:
					coefc = 255*inv_dz*(vx_center.z()-zmin);
					vx_color = TColor(coefc*general_color.R, coefc*general_color.G, coefc*general_color.B, 255.0*general_color.A);
					break;

				case COctoMapVoxels::COLOR_FROM_OCCUPANCY:
					coefc = 240*(1-occ) + 15;
					vx_color = TColor(coefc*general_color.R, coefc*general_color.G, coefc*general_color.B, 255.0*general_color.A);
					break;

				case COctoMapVoxels::TRANSPARENCY_FROM_OCCUPANCY:
					coeft = 255 - 510*(1-occ);
					if (coeft < 0) {	coeft = 0; }
					vx_color = TColor(255*general_color.R, 255*general_color.G, 255*general_color.B, coeft);
					break;

				case COctoMapVoxels::TRANS_AND_COLOR_FROM_OCCUPANCY:
					coefc = 240*(1-occ) + 15;
					vx_color = TColor(coefc*general_color.R, coefc*general_color.G, coefc*general_color.B, 50);
					break;

				case COctoMapVoxels::MIXED:
					coefc = 255*inv_dz*(vx_center.z()-zmin);
					coeft = 255 - 510*(1-occ);
					if (coeft < 0) {	coeft = 0; }
					vx_color = TColor(coefc*general_color.R, coefc*general_color.G, coefc*general_color.B, coeft);
					break;

				default:
					THROW_EXCEPTION("Unknown coloring scheme!")
				}

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

	// if we use transparency, sort cubes by "Z" as an approximation to far-to-near render ordering:
	if (gl_obj.isCubeTransparencyEnabled())
		gl_obj.sort_voxels_by_z();

	// Set bounding box:
	{
		mrpt::math::TPoint3D bbmin, bbmax;
		this->getMetricMin(bbmin.x,bbmin.y,bbmin.z);
		this->getMetricMax(bbmax.x,bbmax.y,bbmax.z);
		gl_obj.setBoundingBox(bbmin, bbmax);
	}

}
