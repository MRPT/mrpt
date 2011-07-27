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

#include <mrpt/maps.h>  // Precompiled header



#include <mrpt/slam/CHeightGridMap2D.h>
#include <mrpt/slam/CObservationGasSensors.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CSimplePointsMap.h>
#include <mrpt/system/os.h>
#include <mrpt/math/utils.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/color_maps.h>
#include <mrpt/opengl/CMesh.h>
#include <mrpt/opengl/CPointCloudColoured.h>

using namespace mrpt::slam;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CHeightGridMap2D, CMetricMap,mrpt::slam)


bool mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH = true;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CHeightGridMap2D::CHeightGridMap2D(
	TMapRepresentation	mapType,
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolution ) :
		CDynamicGrid<THeightGridmapCell>( x_min,x_max,y_min,y_max,resolution ),
		insertionOptions(),
		m_mapType(mapType)
{
}

/*---------------------------------------------------------------
						clear
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::internal_clear()
{
	fill( THeightGridmapCell() );
}

/*---------------------------------------------------------------
						isEmpty
  ---------------------------------------------------------------*/
bool  CHeightGridMap2D::isEmpty() const
{
	return false;
}

/*---------------------------------------------------------------
						insertObservation
  ---------------------------------------------------------------*/
bool  CHeightGridMap2D::internal_insertObservation(
	const CObservation	*obs,
	const CPose3D			*robotPose )
{
	MRPT_START

	CPose2D		robotPose2D;
	CPose3D		robotPose3D;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if ( CLASS_ID( CObservation2DRangeScan )==obs->GetRuntimeClass())
	{
		/********************************************************************
					OBSERVATION TYPE: CObservation2DRangeScan
		********************************************************************/
		const CObservation2DRangeScan	*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Create points map, if not created yet:
		CPointsMap::TInsertionOptions	opts;
		opts.minDistBetweenLaserPoints = insertionOptions.minDistBetweenPointsWhenInserting;
		const CPointsMap	*thePoints = o->buildAuxPointsMap<mrpt::slam::CPointsMap>( &opts );

		// And rotate to the robot pose:
		CSimplePointsMap	thePointsMoved;
		thePointsMoved.changeCoordinatesReference( *thePoints, robotPose3D );

		size_t i, N = thePointsMoved.size();
		float	x,y,z;
		//float	rel_x,rel_y,rel_z;

		// First compute the bounding box:
/*		float	min_x,max_x, min_y,max_y, min_z, max_z;
		thePointsMoved.boundingBox( min_x,max_x,min_y,max_y, min_z,max_z );

		// Resize grid if necessary:
		const THeightGridmapCell	dummyCell;
		resize( min_x,max_x,min_y, max_y, dummyCell, 3 );
*/

		// Insert z's using the selected method:
//		const bool	doWindow = m_mapType==mrSlidingWindow;
//		const mrpt::system::TTimeStamp	tim = o->timestamp;

		for (i=0;i<N;i++)
		{
			thePointsMoved.getPoint(i, x,y,z);
			//thePoints->getPoint(i,rel_x,rel_y,rel_z);

			THeightGridmapCell *cell = cellByPos(x,y);
			if(!cell) continue; // Out of the map: Ignore if we've not resized before.

			//if (!insertionOptions.filterByHeight || (rel_z>=insertionOptions.z_min && rel_z<=insertionOptions.z_max ) )
			if (!insertionOptions.filterByHeight || (z>=insertionOptions.z_min && z<=insertionOptions.z_max ) )
			{
				cell->u += z;
				cell->v += z*z;
				if (!cell->w)
				{
					cell->h = z;	// First observation
					cell->w = 1;
				}
				else
				{
					float W = cell->w++;	// W = N-1
					cell->h = (cell->h*W + z)/cell->w;
					if (W > 0)
						cell->var = 1/(W) * (cell->v - pow(cell->u,2)/cell->w);
				}

				//if (doWindow)
				//	cell->history_Zs.insert( pair<TTimeStamp,float>(tim, z));

			} // end if really inserted
		} // end for i

		return true;	// Done!

	} // end if "CObservationGasSensors"

	return false;

	MRPT_END
}


/*---------------------------------------------------------------
						computeObservationLikelihood
  ---------------------------------------------------------------*/
double	 CHeightGridMap2D::computeObservationLikelihood(
	const CObservation		*obs,
	const CPose3D			&takenFrom )
{
	MRPT_UNUSED_PARAM(obs);MRPT_UNUSED_PARAM(takenFrom);

    THROW_EXCEPTION("Not implemented yet!");
}

/*---------------------------------------------------------------
  Implements the writing to a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CHeightGridMap2D::writeToStream(CStream &out, int *version) const
{
	if (version)
		*version = 1;
	else
	{
		uint32_t	n;

		// Save the dimensions of the grid:
		out << m_x_min << m_x_max << m_y_min << m_y_max;
		out << m_resolution;
		out << static_cast<uint32_t>(m_size_x) << static_cast<uint32_t>(m_size_y);

		// To assure compatibility: The size of each cell:
		n = static_cast<uint32_t>(sizeof( THeightGridmapCell ));
		out << n;

		// Save the map contents:
		n = static_cast<uint32_t>(m_map.size());
		out << n;
		for (vector<THeightGridmapCell>::const_iterator it=m_map.begin();it!=m_map.end();++it)
			out << it->h << it->w; // This was removed in version 1: << it->history_Zs;

		// Save the insertion options:
		out << uint8_t(m_mapType);

		out << insertionOptions.filterByHeight
			<< insertionOptions.z_min
			<< insertionOptions.z_max;
	}
}

/*---------------------------------------------------------------
  Implements the reading from a CStream capability of CSerializable objects
 ---------------------------------------------------------------*/
void  CHeightGridMap2D::readFromStream(CStream &in, int version)
{
	switch(version)
	{
	case 0:
	case 1:
		{
			uint32_t	n,i,j;

			// Load the dimensions of the grid:
			in >> m_x_min >> m_x_max >> m_y_min >> m_y_max;
			in >> m_resolution;
			in >> i >> j;
			m_size_x = i;
			m_size_y = j;

			// To assure compatibility: The size of each cell:
			in >> n;
			ASSERT_( n == static_cast<uint32_t>( sizeof( THeightGridmapCell ) ));

			// Save the map contents:
			in >> n;
			m_map.resize(n);
			for (vector<THeightGridmapCell>::iterator it=m_map.begin();it!=m_map.end();++it)
			{
				in >> it->h >> it->w;
				// Data member in version 0:
				if (version==0)
				{
					std::multimap<mrpt::system::TTimeStamp,float>	history_Zs;
					in >> history_Zs; // Discarded now...
				}
			}

			// Insertion options:
			uint8_t	ty;
			in  >> ty;
			m_mapType = TMapRepresentation(ty);

			in  >> insertionOptions.filterByHeight
				>> insertionOptions.z_min
				>> insertionOptions.z_max;

		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};

}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CHeightGridMap2D::TInsertionOptions::TInsertionOptions() :
	filterByHeight				( false ),
	z_min						( -0.5  ),
	z_max						(  0.5  ),
	minDistBetweenPointsWhenInserting ( 0 )
{
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CHeightGridMap2D::TInsertionOptions] ------------ \n\n");
	out.printf("filterByHeight                          = %c\n", filterByHeight ? 'y':'n');
	out.printf("z_min                                   = %f\n", z_min);
	out.printf("z_max                                   = %f\n", z_max);
	out.printf("minDistBetweenPointsWhenInserting       = %f\n", minDistBetweenPointsWhenInserting);
	out.printf("\n");
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR( filterByHeight,	bool, iniFile, section )
	MRPT_LOAD_CONFIG_VAR( z_min,			float, iniFile, section )
	MRPT_LOAD_CONFIG_VAR( z_max,			float, iniFile, section )
	MRPT_LOAD_CONFIG_VAR( minDistBetweenPointsWhenInserting, float, iniFile, section )
}

/*---------------------------------------------------------------
					saveMetricMapRepresentationToFile
  ---------------------------------------------------------------*/
void  CHeightGridMap2D::saveMetricMapRepresentationToFile(
	const std::string	&filNamePrefix ) const
{
	std::string		fil;

	// Text matrix:
	saveToTextFile( filNamePrefix + std::string("_mean.txt") );
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CHeightGridMap2D::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	if (m_disableSaveAs3DObject)
		return;

	if (mrpt::global_settings::HEIGHTGRIDMAP_EXPORT3D_AS_MESH)
	{
		opengl::CMeshPtr	mesh = opengl::CMesh::Create();

		mesh->setGridLimits(m_x_min,m_x_max, m_y_min, m_y_max);

		mesh->setColor(0.4, 0.4, 0.4 );

		mesh->enableWireFrame(true);
		mesh->enableColorFromZ(true, cmJET);

		CMatrixFloat Z,mask;
		/*mesh->getZ(Z);

		mesh->getMask(mask);*/

		Z.setSize( m_size_x, m_size_y );
		mask.setSize( m_size_x, m_size_y );

		for (size_t x=0;x<m_size_x;x++)
		{
			for (size_t y=0;y<m_size_y;y++)
			{
				const THeightGridmapCell *c = cellByIndex(x,y);
				ASSERTDEB_(c);
				Z.set_unsafe( x, y, c->h );
				mask.set_unsafe( x, y, c->w ? 1:0 );
			}
		}
		mesh->setZ(Z);
		mesh->setMask(mask);

		outObj->insert( mesh );
	}
	else
	{
		// As points:
		mrpt::opengl::CPointCloudColouredPtr obj = mrpt::opengl::CPointCloudColoured::Create();
		obj->setPointSize(2);

		// Find min/max:
		float z_min,z_max;
		float K;
		if (this->getMinMaxHeight(z_min,z_max))
		     K = 1.0f/(z_max-z_min);
		else K = 1.0f;

		obj->reserve(m_size_x*m_size_y);
		for (size_t x=0;x<m_size_x;x++)
			for (size_t y=0;y<m_size_y;y++)
			{
				const THeightGridmapCell *c = cellByIndex(x,y);
				ASSERTDEB_(c)
				if (c->w)
				{
					float r,g,b;
					const float col_idx = (c->h-z_min)*K;
					colormap(
						cmGRAYSCALE, //cmJET,
						col_idx, r,g,b );
					obj->push_back( idx2x(x),idx2y(y), c->h, r,g,b );
				}
			}

		outObj->insert( obj );
	}
}

/** Computes the minimum and maximum height in the grid.
  * \return False if there is no observed cell yet.
  */
bool CHeightGridMap2D::getMinMaxHeight(float &z_min, float &z_max) const
{
	bool any = false;
	z_min = z_max = 0;
	for (size_t x=0;x<m_size_x;x++)
		for (size_t y=0;y<m_size_y;y++)
		{
			const THeightGridmapCell *c = cellByIndex(x,y);
			ASSERTDEB_(c)
			if (c->w)
			{
				if (!any)
				{
					// First:
					any = true;
					z_min = z_max = c->h;
				}
				else
				{
					mrpt::utils::keep_max(z_max , c->h );
					mrpt::utils::keep_min(z_min , c->h );
				}
			}
		}
	return any;
}


/*---------------------------------------------------------------
  Computes the ratio in [0,1] of correspondences between "this" and the "otherMap" map, whose 6D pose relative to "this" is "otherMapPose"
 *   In the case of a multi-metric map, this returns the average between the maps. This method always return 0 for grid maps.
 * \param  otherMap					  [IN] The other map to compute the matching with.
 * \param  otherMapPose				  [IN] The 6D pose of the other map as seen from "this".
 * \param  minDistForCorr			  [IN] The minimum distance between 2 non-probabilistic map elements for counting them as a correspondence.
 * \param  minMahaDistForCorr		  [IN] The minimum Mahalanobis distance between 2 probabilistic map elements for counting them as a correspondence.
 *
 * \return The matching ratio [0,1]
 * \sa computeMatchingWith2D
  --------------------------------------------------------------- */
float  CHeightGridMap2D::compute3DMatchingRatio(
		const CMetricMap						*otherMap,
		const CPose3D							&otherMapPose,
		float									minDistForCorr,
		float									minMahaDistForCorr
		) const
{
	MRPT_UNUSED_PARAM(otherMap);
	MRPT_UNUSED_PARAM(otherMapPose);
	MRPT_UNUSED_PARAM(minDistForCorr);
	MRPT_UNUSED_PARAM(minMahaDistForCorr);

	return 0;
}

/*---------------------------------------------------------------
					auxParticleFilterCleanUp
  Gets the intersection between a 3D line and a Height Grid map
   (taking into account the different heights of each individual cell).
 ---------------------------------------------------------------*/
bool CHeightGridMap2D::intersectLine3D(const TLine3D &ray, TObject3D &obj) const
{
	MRPT_START

	obj = TObject3D();

	float z_min,z_max;
	if (!getMinMaxHeight(z_min,z_max))
		return false;


	// 1st: intersections with 2 horizontal planes at the grid Z limits:
	const TPlane horz_plane_above(TPoint3D(0,0,z_max+1),TPoint3D(1,0,z_max+1),TPoint3D(0,1,z_max+1));
	const TPlane horz_plane_below(TPoint3D(0,0,z_min-1),TPoint3D(1,0,z_min-1),TPoint3D(0,1,z_min-1));
	TPoint3D pt_ab,pt_be;
	{
		TObject3D int_ab,int_be;
		intersect(ray,horz_plane_above, int_ab);
		intersect(ray,horz_plane_below, int_be);

		if (!int_ab.getPoint(pt_ab) || !int_be.getPoint(pt_be))
			return false;
	}

	// Now, go from pt_ab -> pt_be doing "ray-tracing" and find the collision with a cell:
	TPoint3D  pt = pt_ab;
	TPoint3D  Apt = pt_be-pt_ab;
	const double totalDist = Apt.norm();
	if (totalDist==0) return false;
	// The step:
	Apt*= this->m_resolution * 0.99/totalDist;

	TPoint3D Apt_half=Apt;
	Apt_half*=0.5;

	const size_t N = ceil(totalDist/m_resolution);

	for (size_t i=0;i<N;i++)
	{
		// Mid point between this and next step:
		const TPoint3D  testPt = pt + Apt_half;
		// get its height in the grid:
		const THeightGridmapCell *cell = cellByPos(testPt.x,testPt.y);
		if (cell && cell->w)
		{
			// Do we go thru the cell?
			if ( cell->h >= std::min(pt.z,pt.z+Apt.z) && cell->h < std::max(pt.z,pt.z+Apt.z) )
			{
				// yes:
				TPoint3D colPt(testPt.x,testPt.y,cell->h);
				obj = TObject3D(colPt);
				return true;
			}
		}
		pt+=Apt;
	}

	// No collision found!
	return false;

	// None found:
	MRPT_END
}


/** Return the number of cells with at least one height data inserted. */
size_t CHeightGridMap2D::countObservedCells() const
{
	switch (m_mapType)
	{
	case mrSimpleAverage:
		{
			size_t obsCells = 0;
			const size_t N = m_map.size();
			for (size_t i=0;i<N;i++)
				if (m_map[i].w)
					obsCells++;
			return obsCells;
		}
		break;
	default: THROW_EXCEPTION("countObservedCells() not implemented for this mapType (!?)")
	};
}
