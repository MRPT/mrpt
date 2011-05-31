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

#include <mrpt/reactivenav.h>  // Precomp header

#include <mrpt/math/geometry.h>
#include <mrpt/utils/CTicTac.h>

using namespace std;
using namespace mrpt;
using namespace mrpt::reactivenav;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;

/*---------------------------------------------------------------
					build_PTG_collision_grids
  ---------------------------------------------------------------*/
void mrpt::reactivenav::build_PTG_collision_grids(
	std::vector<CParameterizedTrajectoryGenerator*>	PTGs,
	const mrpt::math::CPolygon						&robotShape,
	const std::string								&cacheFilesPrefix,
	bool											verbose
	)
{
	MRPT_START

	const size_t nVerts = robotShape.verticesCount();

	if (verbose) 
		cout << endl << "[build_PTG_collision_grids] Starting... *** THIS MAY TAKE A WHILE, BUT MUST BE COMPUTED ONLY ONCE!! **" << endl;

	for (size_t nPT=0;nPT<PTGs.size();nPT++)
	{
		utils::CTicTac		tictac;

		if (verbose)
			cout << "Computing collision cells for PTG[" << nPT << "]...";

		ASSERT_(PTGs[nPT])
		CParameterizedTrajectoryGenerator  *PT = PTGs[nPT];

		tictac.Tic();

		// Reservar memoria para todos los puntos:
		PT->allocMemForVerticesData( nVerts );

		const size_t nPaths = PT->getAlfaValuesCount();

		for (size_t k=0;k<nPaths;k++)
		{
			const size_t nPointsInPath = PT->getPointsCountInCPath_k( k );

			for (size_t n=0;n<nPointsInPath;n++)
			{
				float x   = PT->GetCPathPoint_x( k,n );
				float y   = PT->GetCPathPoint_y( k,n );
				float phi = PT->GetCPathPoint_phi( k,n );

				for (size_t m = 0;m<nVerts;m++)
				{
					float vx = x + cos(phi)*robotShape.GetVertex_x(m)-sin(phi)*robotShape.GetVertex_y(m);
					float vy = y + sin(phi)*robotShape.GetVertex_x(m)+cos(phi)*robotShape.GetVertex_y(m);
					PT->setVertex_xy(k,n,m, vx, vy );
				}       // for v
			}       // for n
		} // for k


		// Check for collisions between the robot shape and the grid cells:
		// ----------------------------------------------------------------------------
		const size_t Ki = PT->getAlfaValuesCount();

		std::string auxStr = format( "%s_PTG%03u.dat.gz",cacheFilesPrefix.c_str(),static_cast<unsigned int>(nPT));

		// Load the cached version, if possible
		if ( PT->LoadColGridsFromFile( auxStr ) )
		{
			if (verbose)
				cout << "loaded from file OK" << endl;
		}
		else
		{
			// RECOMPUTE THE COLLISION GRIDS:
			// ---------------------------------------
			for (size_t k=0;k<Ki;k++)
			{
				const size_t nPoints = PT->getPointsCountInCPath_k(k);
				ASSERT_(nPoints>1)

				for (size_t n=0;n<(nPoints-1);n++)
				{
					// Check for collisions between an obstacle in a grid cell and 
					//  the segment "s" between time steps "n" and "n+1"
					for (size_t m=0;m<nVerts;m++)
					{
						float          v1n_x,  v1n_y;
						float          v2n_x,  v2n_y;
						float          v1n1_x, v1n1_y;
						float          v2n1_x, v2n1_y;

						float          minx,maxx,miny,maxy;

						v1n_x = PT->getVertex_x(k,n,m);
						v1n_y = PT->getVertex_y(k,n,m);

						v1n1_x = PT->getVertex_x(k,n+1,m);
						v1n1_y = PT->getVertex_y(k,n+1,m);

						v2n_x = PT->getVertex_x(k,n,(m+1) % nVerts);
						v2n_y = PT->getVertex_y(k,n,(m+1) % nVerts);

						v2n1_x = PT->getVertex_x(k,n+1,(m+1) % nVerts);
						v2n1_y = PT->getVertex_y(k,n+1,(m+1) % nVerts);

						minx=min( v1n_x, min( v2n_x, min( v1n1_x, v2n1_x ) ) );
						maxx=max( v1n_x, max( v2n_x, max( v1n1_x, v2n1_x ) ) );
						miny=min( v1n_y, min( v2n_y, min( v1n1_y, v2n1_y ) ) );
						maxy=max( v1n_y, max( v2n_y, max( v1n1_y, v2n1_y ) ) );

						// Get the range of the cell grid that is affected by the movement
						//  of this segment of the robot, and just check collisions at that area:
						const int ix_min = PT->m_collisionGrid.x2idx(minx);
						const int iy_min = PT->m_collisionGrid.y2idx(miny);
						const int ix_max = PT->m_collisionGrid.x2idx(maxx);
						const int iy_max = PT->m_collisionGrid.y2idx(maxy);

						const float res_mid = .5f * PT->m_collisionGrid.getResolution();

						for (int ix=ix_min;ix<ix_max;ix++)
						{
							const float cx_mid = res_mid + PT->m_collisionGrid.idx2x(ix);
							
							for (int iy=iy_min;iy<iy_max;iy++)
							{
								const float cy_mid = res_mid + PT->m_collisionGrid.idx2y(iy);

								if ( mrpt::math::pointIntoQuadrangle( 
									cx_mid,cy_mid, 
									v1n_x, v1n_y, v2n_x, v2n_y, v2n1_x, v2n1_y, v1n1_x, v1n1_y ) )
								{
									// Colision!! Update cell info:
									PT->m_collisionGrid.updateCellInfo(
										ix,iy,
									    k,							// The path (Alfa value)
									    PT->GetCPathPoint_d(k, n+1) // The collision distance
									);

								}
							}	// for iy
						}	// for ix
					}	// for m
				} // n
				
				if (verbose) 
					cout << k << "/" << Ki << ",";
			} // k

			if (verbose) 
				cout << format("Done! [%.03f sec]\n",tictac.Tac() );

			// save it to the cache file for the next run:
			PT->SaveColGridsToFile( auxStr );

		}	// "else" recompute all PTG
	}	// for nPT

	MRPT_END
}



