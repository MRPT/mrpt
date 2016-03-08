/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "nav-precomp.h" // Precomp header

#include <mrpt/nav/tpspace/motion_planning_utils.h>
#include <mrpt/math/geometry.h>
#include <mrpt/utils/CTicTac.h>

using namespace std																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																																		;
using namespace mrpt;
using namespace mrpt::nav;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;

/*---------------------------------------------------------------
					build_PTG_collision_grids
  ---------------------------------------------------------------*/
void mrpt::nav::build_PTG_collision_grids(
	CParameterizedTrajectoryGenerator * PT,
	const mrpt::math::CPolygon        & robotShape,
	const std::string                 & cacheFilename,
	const bool                          verbose
	)
{
	MRPT_START

	if (verbose)
		cout << endl << "[build_PTG_collision_grids] Starting... *** THIS MAY TAKE A WHILE, BUT MUST BE COMPUTED ONLY ONCE!! **" << endl;

	utils::CTicTac		tictac;

	if (verbose)
		cout << "Computing collision cells for PTG '" << cacheFilename << "'...";

	ASSERT_(PT)

	tictac.Tic();

	//const size_t nPaths = PT->getAlfaValuesCount();

	// Check for collisions between the robot shape and the grid cells:
	// ----------------------------------------------------------------------------
	const size_t Ki = PT->getAlfaValuesCount();

	//Collision grid

	// Load the cached version, if possible	
	if (PT->LoadColGridsFromFile( cacheFilename, robotShape ) )
	{
		if (verbose)
			cout << "CollisonGrid loaded from file OK" << endl;
	}
	else
	{
		// BUGFIX: In case we start reading the file and in the end detected an error,
		//         we must make sure that there's space enough for the grid:
		PT->m_collisionGrid.setSize( -PT->refDistance,PT->refDistance,-PT->refDistance,PT->refDistance,PT->m_collisionGrid.getResolution());

		const int grid_cx_max = PT->m_collisionGrid.getSizeX()-1;
		const int grid_cy_max = PT->m_collisionGrid.getSizeY()-1;

		const size_t nVerts = robotShape.verticesCount();
		std::vector<mrpt::math::TPoint2D> transf_shape(nVerts); // The robot shape at each location

		// RECOMPUTE THE COLLISION GRIDS:
		// ---------------------------------------
		for (size_t k=0;k<Ki;k++)
		{
			const size_t nPoints = PT->getPointsCountInCPath_k(k);
			ASSERT_(nPoints>1)

			for (size_t n=0;n<(nPoints-1);n++)
			{
				// Translate and rotate the robot shape at this C-Space pose:
				const double x   = PT->GetCPathPoint_x( k,n );
				const double y   = PT->GetCPathPoint_y( k,n );
				const double phi = PT->GetCPathPoint_phi( k,n );

				mrpt::math::TPoint2D bb_min(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
				mrpt::math::TPoint2D bb_max(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

				for (size_t m = 0;m<nVerts;m++)
				{
					transf_shape[m].x = x + cos(phi)*robotShape.GetVertex_x(m)-sin(phi)*robotShape.GetVertex_y(m);
					transf_shape[m].y = y + sin(phi)*robotShape.GetVertex_x(m)+cos(phi)*robotShape.GetVertex_y(m);
					mrpt::utils::keep_max( bb_max.x, transf_shape[m].x); mrpt::utils::keep_max( bb_max.y, transf_shape[m].y);
					mrpt::utils::keep_min( bb_min.x, transf_shape[m].x); mrpt::utils::keep_min( bb_min.y, transf_shape[m].y);
				}

				// Robot shape polygon:
				const mrpt::math::TPolygon2D poly(transf_shape);

				// Get the range of cells that may collide with this shape:
				const int ix_min = std::max(0,PT->m_collisionGrid.x2idx(bb_min.x)-1);
				const int iy_min = std::max(0,PT->m_collisionGrid.y2idx(bb_min.y)-1);
				const int ix_max = std::min(PT->m_collisionGrid.x2idx(bb_max.x)+1,grid_cx_max);
				const int iy_max = std::min(PT->m_collisionGrid.y2idx(bb_max.y)+1,grid_cy_max);

				for (int ix=ix_min;ix<ix_max;ix++)
				{
					const float cx = PT->m_collisionGrid.idx2x(ix);

					for (int iy=iy_min;iy<iy_max;iy++)
					{
						const float cy = PT->m_collisionGrid.idx2y(iy);

						if ( poly.contains( TPoint2D(cx,cy) ) )
						{
							// Colision!! Update cell info:
							const float d = PT->GetCPathPoint_d(k,n);
							PT->m_collisionGrid.updateCellInfo(ix  ,iy  ,  k,d);
							PT->m_collisionGrid.updateCellInfo(ix-1,iy  ,  k,d);
							PT->m_collisionGrid.updateCellInfo(ix  ,iy-1,  k,d);
							PT->m_collisionGrid.updateCellInfo(ix-1,iy-1,  k,d);
						}
						
					}	// for iy
				}	// for ix

			} // n

			if (verbose)
				cout << k << "/" << Ki << ",";
		} // k

		if (verbose)
			cout << format("Done! [%.03f sec]\n",tictac.Tac() );

		// save it to the cache file for the next run:
		PT->SaveColGridsToFile( cacheFilename, robotShape );

	}	// "else" recompute all PTG

	//Clearance grid
	if (PT->LoadCleGridsFromFile( cacheFilename, robotShape ) )
	{
		if (verbose)
			cout << "ClearanceGrid loaded from file OK" << endl;
	}
	else
	{
		// BUGFIX: In case we start reading the file and in the end detected an error,
		//         we must make sure that there's space enough for the grid:
		PT->m_clearanceGrid.setSize( -PT->refDistance,PT->refDistance,-PT->refDistance,PT->refDistance,PT->m_clearanceGrid.getResolution());

		const int grid_cx_max = PT->m_clearanceGrid.getSizeX()-1;
		const int grid_cy_max = PT->m_clearanceGrid.getSizeY()-1;

		const size_t nVerts = robotShape.verticesCount();
		std::vector<mrpt::math::TPoint2D> transf_shape(nVerts); // The robot shape at each location

		// RECOMPUTE THE COLLISION GRIDS:
		// ---------------------------------------
		for (size_t k=0;k<Ki;k++)
		{
			const size_t nPoints = PT->getPointsCountInCPath_k(k);
			ASSERT_(nPoints>1)

			for (size_t n=0;n<(nPoints-1);n++)
			{
				// Translate and rotate the robot shape at this C-Space pose:
				const double x   = PT->GetCPathPoint_x( k,n );
				const double y   = PT->GetCPathPoint_y( k,n );
				const double phi = PT->GetCPathPoint_phi( k,n );

				mrpt::math::TPoint2D bb_min(std::numeric_limits<double>::max(),std::numeric_limits<double>::max());
				mrpt::math::TPoint2D bb_max(-std::numeric_limits<double>::max(),-std::numeric_limits<double>::max());

				for (size_t m = 0;m<nVerts;m++)
				{
					transf_shape[m].x = x + cos(phi)*robotShape.GetVertex_x(m)-sin(phi)*robotShape.GetVertex_y(m);
					transf_shape[m].y = y + sin(phi)*robotShape.GetVertex_x(m)+cos(phi)*robotShape.GetVertex_y(m);
					mrpt::utils::keep_max( bb_max.x, transf_shape[m].x); mrpt::utils::keep_max( bb_max.y, transf_shape[m].y);
					mrpt::utils::keep_min( bb_min.x, transf_shape[m].x); mrpt::utils::keep_min( bb_min.y, transf_shape[m].y);
				}

				// Robot shape polygon:
				const mrpt::math::TPolygon2D poly(transf_shape);

				// Get the range of cells that may collide with this shape:
				const int ix_min = std::max(0,PT->m_clearanceGrid.x2idx(bb_min.x)-1);
				const int iy_min = std::max(0,PT->m_clearanceGrid.y2idx(bb_min.y)-1);
				const int ix_max = std::min(PT->m_clearanceGrid.x2idx(bb_max.x)+1,grid_cx_max);
				const int iy_max = std::min(PT->m_clearanceGrid.y2idx(bb_max.y)+1,grid_cy_max);

				for (int ix=ix_min;ix<ix_max;ix++)
				{
					const float cx = PT->m_clearanceGrid.idx2x(ix);

					for (int iy=iy_min;iy<iy_max;iy++)
					{
						const float cy = PT->m_clearanceGrid.idx2y(iy);						

						//Updating the clearance grid.. (currently for all n.. will be modified later)

						//calculates distance from the polygon to the cell
						const float dist = poly.distance(mrpt::math::TPoint2D(cx,cy));

						PT->m_clearanceGrid.updateCellInfo(ix,iy,k,n,dist);
						PT->m_clearanceGrid.updateCellInfo(ix-1,iy,k,n,dist);
						PT->m_clearanceGrid.updateCellInfo(ix,iy-1,k,n,dist);
						PT->m_clearanceGrid.updateCellInfo(ix-1,iy-1,k,n,dist);
					}	// for iy
				}	// for ix

			} // n

			if (verbose)
				cout << k << "/" << Ki << ",";
		} // k

		if (verbose)
			cout << format("Done! [%.03f sec]\n",tictac.Tac() );

		// save it to the cache file for the next run:
		PT->SaveCleGridsToFile( cacheFilename, robotShape );

	}	// "else" recompute all PTG

	MRPT_END
}

