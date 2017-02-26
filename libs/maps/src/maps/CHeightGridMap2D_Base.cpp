/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/CHeightGridMap2D_Base.h>
#include <mrpt/math/geometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/maps/CSimplePointsMap.h>

using namespace mrpt::maps;
using namespace std;

CHeightGridMap2D_Base::TPointInsertParams::TPointInsertParams() :
	pt_z_std (0.0),
	update_map_after_insertion(true)
{
}

CHeightGridMap2D_Base::CHeightGridMap2D_Base()
{
}

CHeightGridMap2D_Base::~CHeightGridMap2D_Base()
{
}

bool CHeightGridMap2D_Base::getMinMaxHeight(float &z_min, float &z_max) const
{
	const size_t size_x = dem_get_size_x();
	const size_t size_y = dem_get_size_y();

	bool any = false;
	z_min = z_max = 0;
	for (size_t x=0;x<size_x;x++)
		for (size_t y=0;y<size_y;y++)
		{
			double z;
			if (dem_get_z_by_cell(x,y,z))
			{
				if (!any) 
				{
					// First:
					any = true;
					z_min = z_max = z;
				}
				else
				{
					mrpt::utils::keep_max(z_max , z );
					mrpt::utils::keep_min(z_min , z );
				}
			}
		}
	return any;
}



bool CHeightGridMap2D_Base::intersectLine3D(const mrpt::math::TLine3D &ray, mrpt::math::TObject3D &obj) const
{
	using namespace mrpt::math;

	MRPT_START

	obj = TObject3D();

	const double resolution = dem_get_resolution();
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
	Apt*= resolution * 0.99/totalDist;

	TPoint3D Apt_half=Apt;
	Apt_half*=0.5;

	const size_t N = ceil(totalDist/resolution);

	for (size_t i=0;i<N;i++)
	{
		// Mid point between this and next step:
		const TPoint3D  testPt = pt + Apt_half;
		// get its height in the grid:
		double pt_z;
		if (dem_get_z(testPt.x,testPt.y, pt_z) ) 
		{
			// Do we go thru the cell?
			if ( pt_z >= std::min(pt.z,pt.z+Apt.z) && pt_z < std::max(pt.z,pt.z+Apt.z) )
			{
				// yes:
				TPoint3D colPt(testPt.x,testPt.y,pt_z);
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

bool CHeightGridMap2D_Base::dem_internal_insertObservation(const mrpt::obs::CObservation *obs, const mrpt::poses::CPose3D *robotPose)
{
	using namespace mrpt::poses;
	using namespace mrpt::obs;

	MRPT_START

	CPose3D robotPose3D;  // Default: 0,0,0

	if (robotPose)
		robotPose3D = (*robotPose);

	// Points to insert:
	CSimplePointsMap	thePointsMoved;

	if ( IS_CLASS(obs, CObservation2DRangeScan ))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservation2DRangeScan
		********************************************************************/
		const CObservation2DRangeScan	*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Create points map, if not created yet:
		CPointsMap::TInsertionOptions	opts;
		const CPointsMap	*thePoints = o->buildAuxPointsMap<mrpt::maps::CPointsMap>( &opts );

		// And rotate to the robot pose:
		thePointsMoved.changeCoordinatesReference( *thePoints, robotPose3D );
	}
	else
	if ( IS_CLASS(obs, CObservationVelodyneScan ))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationVelodyneScan
		********************************************************************/
		const CObservationVelodyneScan *o = static_cast<const CObservationVelodyneScan*>( obs );

		// Create points map, if not created yet:
		thePointsMoved.loadFromVelodyneScan(*o,&robotPose3D);
	}

	// Factorized insertion of points, for different observation classes:
	if (!thePointsMoved.empty())
	{
		TPointInsertParams pt_params;
		pt_params.update_map_after_insertion = false; // update only once at end

		const size_t N = thePointsMoved.size();
		for (size_t i=0;i<N;i++)
		{
			float x,y,z;
			thePointsMoved.getPoint(i, x,y,z);
			insertIndividualPoint(x,y,z,pt_params);
		} // end for i
		this->dem_update_map();
		return true; // Done, new points inserted
	}
	return false; // No insertion done
	MRPT_END
}
