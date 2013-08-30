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

#include <mrpt/slam/CColouredOctoMap.h>
#include <mrpt/slam/CPointsMap.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>

#include <mrpt/opengl/COctoMapVoxels.h>
#include <mrpt/opengl/COpenGLScene.h>

#include <mrpt/system/filesystem.h>
#include <mrpt/utils/CMemoryChunk.h>
#include <mrpt/utils/CFileOutputStream.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace mrpt::opengl;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CColouredOctoMap, CMetricMap,mrpt::slam)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CColouredOctoMap::CColouredOctoMap(const double resolution) :
	COctoMapBase<octomap::ColorOcTree,octomap::ColorOcTreeNode>(resolution),
	m_colour_method(INTEGRATE)
{
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CColouredOctoMap::~CColouredOctoMap()
{
}

/*---------------------------------------------------------------
					writeToStream
   Implements the writing to a CStream capability of
				CSerializable objects
  ---------------------------------------------------------------*/
void  CColouredOctoMap::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		this->likelihoodOptions.writeToStream(out);
		this->renderingOptions.writeToStream(out);  // Added in v1

		CMemoryChunk chunk;
		const string	tmpFil = mrpt::system::getTempFileName();
		const_cast<octomap::ColorOcTree*>(&m_octomap)->writeBinary(tmpFil);
		chunk.loadBufferFromFile(tmpFil);
		mrpt::system::deleteFile(tmpFil);
		out << chunk;

	}
}

/*---------------------------------------------------------------
					readFromStream
   Implements the reading from a CStream capability of
      CSerializable objects
  ---------------------------------------------------------------*/
void  CColouredOctoMap::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			this->likelihoodOptions.readFromStream(in);
			if (version>=1) this->renderingOptions.readFromStream(in);

			this->clear();

			CMemoryChunk chunk;
			in >> chunk;

			if (chunk.getTotalBytesCount())
			{
				const string	tmpFil = mrpt::system::getTempFileName();
				if (!chunk.saveBufferToFile( tmpFil ) ) THROW_EXCEPTION("Error saving temporary file");
				m_octomap.readBinary(tmpFil);
				mrpt::system::deleteFile( tmpFil );
			}

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	};

}

/*---------------------------------------------------------------
				insertObservation
 ---------------------------------------------------------------*/
bool CColouredOctoMap::internal_insertObservation(const CObservation *obs,const CPose3D *robotPose)
{
	octomap::point3d     sensorPt;
	octomap::Pointcloud  scan;

	if (!internal_build_PointCloud_for_observation(obs,robotPose, sensorPt, scan))
		return false; // Nothing to do.

	// Insert rays:
	m_octomap.insertScan(scan, sensorPt, insertionOptions.maxrange, insertionOptions.pruning);
	octomap::point3d a = scan.getPoint(1);

	return true;
}

/** Get the RGB colour of a point
* \return false if the point is not mapped, in which case the returned colour is undefined. */
bool CColouredOctoMap::getPointColour(const float x, const float y, const float z, char& r, char& g, char& b) const
{
    octomap::OcTreeKey key;
	if (m_octomap.coordToKeyChecked(octomap::point3d(x,y,z), key))
	{
		octomap::ColorOcTreeNode *node = m_octomap.search(key,0 /*depth*/);
		if (!node) return false;

		r = node->getColor().r;
		g = node->getColor().g;
		b = node->getColor().b;
		return true;
	}
	else return false;
}

/** Manually update the colour of the voxel at (x,y,z) */
void CColouredOctoMap::updateVoxelColour(const double x, const double y, const double z, const char r, const char g, const char b)
{
	switch (m_colour_method) {
	case INTEGRATE:
		m_octomap.integrateNodeColor(x,y,z,r,g,b);
		break;
	case SET:
		m_octomap.setNodeColor(x,y,z,r,g,b);
		break;
	case AVERAGE:
		m_octomap.averageNodeColor(x,y,z,r,g,b);
		break;
	default: 
		THROW_EXCEPTION("Invalid value found for 'm_colour_method'")
	}
}


/** Builds a renderizable representation of the octomap as a mrpt::opengl::COctoMapVoxels object. */
void CColouredOctoMap::getAsOctoMapVoxels(mrpt::opengl::COctoMapVoxels &gl_obj) const
{
	// Go thru all voxels:

	//OcTreeVolume voxel; // current voxel, possibly transformed
	octomap::ColorOcTree::tree_iterator it_end = m_octomap.end_tree();

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

	for(octomap::ColorOcTree::tree_iterator it = m_octomap.begin_tree(max_depth);it!=it_end; ++it)
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
				octomap::ColorOcTreeNode::Color node_color = it->getColor();
				vx_color = TColor(node_color.r, node_color.g, node_color.b);

				const size_t vx_set = (m_octomap.isNodeOccupied(*it)) ? VOXEL_SET_OCCUPIED:VOXEL_SET_FREESPACE;

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

