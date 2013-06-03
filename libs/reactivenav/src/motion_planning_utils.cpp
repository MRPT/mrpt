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
		if ( PT->LoadColGridsFromFile( auxStr, robotShape ) )
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
			PT->SaveColGridsToFile( auxStr, robotShape );

		}	// "else" recompute all PTG
	}	// for nPT

	MRPT_END
}

/*---------------------------------------------------------------
			build_PTG_collision_grids for 3D reactive nav.
  ---------------------------------------------------------------*/
void mrpt::reactivenav::build_PTG_collision_grid3D(
	CParameterizedTrajectoryGenerator*				ptg,
	const mrpt::math::CPolygon						&robotShape,
	const unsigned int								&level,
	const unsigned int								&num_ptg,
	bool											verbose
	)
{
	MRPT_START

	const size_t nVerts = robotShape.verticesCount();

	if (verbose)
	{
		cout << endl << "[build_PTG_collision_grids] Starting...  THIS MAY TAKE A WHILE, BUT MUST BE COMPUTED ONLY ONCE!! ";
		cout << endl << "Computing collision cells for PTG[" << num_ptg << "] at level " << level << "...";
	}

	utils::CTicTac		tictac;	
	tictac.Tic();

	// Allocate memory for the points:
	ptg->allocMemForVerticesData( nVerts );

	const size_t nPaths = ptg->getAlfaValuesCount();

	for (size_t k=0;k<nPaths;k++)
	{
		const size_t nPointsInPath = ptg->getPointsCountInCPath_k( k );

		for (size_t n=0;n<nPointsInPath;n++)
		{
			float x   = ptg->GetCPathPoint_x( k,n );
			float y   = ptg->GetCPathPoint_y( k,n );
			float phi = ptg->GetCPathPoint_phi( k,n );

			for (size_t m = 0;m<nVerts;m++)
			{
				float vx = x + cos(phi)*robotShape.GetVertex_x(m)-sin(phi)*robotShape.GetVertex_y(m);
				float vy = y + sin(phi)*robotShape.GetVertex_x(m)+cos(phi)*robotShape.GetVertex_y(m);
				ptg->setVertex_xy(k,n,m, vx, vy );
			}       // for v
		}      // for n
	} // for k


	// Check for collisions between the robot shape and the grid cells:
	// ----------------------------------------------------------------------------
	const size_t Ki = ptg->getAlfaValuesCount();

	std::string auxStr = format( "CollisionGridPTG%02u_LEVEL%02u.dat.gz", num_ptg, level);

	// Load the cached version, if possible
	if ( ptg->LoadColGridsFromFile( auxStr, robotShape ) )
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
			const size_t nPoints = ptg->getPointsCountInCPath_k(k);
			ASSERT_(nPoints>1)

			for (size_t n=0;n<(nPoints-1);n++)
			{
				// Check for collisions between an obstacle in a grid cell and
				//  the segment "s" between time steps "n" and "n+1"
				for (size_t m=0;m<nVerts;m++)
				{
					float   v1n_x,  v1n_y;
					float   v2n_x,  v2n_y;
					float   v1n1_x, v1n1_y;
					float   v2n1_x, v2n1_y;

					float   minx,maxx,miny,maxy;

					v1n_x = ptg->getVertex_x(k,n,m);
					v1n_y = ptg->getVertex_y(k,n,m);

					v1n1_x = ptg->getVertex_x(k,n+1,m);
					v1n1_y = ptg->getVertex_y(k,n+1,m);

					v2n_x = ptg->getVertex_x(k,n,(m+1) % nVerts);
					v2n_y = ptg->getVertex_y(k,n,(m+1) % nVerts);

					v2n1_x = ptg->getVertex_x(k,n+1,(m+1) % nVerts);
					v2n1_y = ptg->getVertex_y(k,n+1,(m+1) % nVerts);

					minx=min( v1n_x, min( v2n_x, min( v1n1_x, v2n1_x ) ) );
					maxx=max( v1n_x, max( v2n_x, max( v1n1_x, v2n1_x ) ) );
					miny=min( v1n_y, min( v2n_y, min( v1n1_y, v2n1_y ) ) );
					maxy=max( v1n_y, max( v2n_y, max( v1n1_y, v2n1_y ) ) );

					// Get the range of the cell grid that is affected by the movement
					// of this segment of the robot, and just check collisions at that area:
					const int ix_min = ptg->m_collisionGrid.x2idx(minx);
					const int iy_min = ptg->m_collisionGrid.y2idx(miny);
					const int ix_max = ptg->m_collisionGrid.x2idx(maxx);
					const int iy_max = ptg->m_collisionGrid.y2idx(maxy);

					const float res_mid = .5f * ptg->m_collisionGrid.getResolution();

					for (int ix=ix_min;ix<ix_max;ix++)
					{
						const float cx_mid = res_mid + ptg->m_collisionGrid.idx2x(ix);

						for (int iy=iy_min;iy<iy_max;iy++)
						{
							const float cy_mid = res_mid + ptg->m_collisionGrid.idx2y(iy);

							if ( mrpt::math::pointIntoQuadrangle(
								cx_mid,cy_mid,
								v1n_x, v1n_y, v2n_x, v2n_y, v2n1_x, v2n1_y, v1n1_x, v1n1_y ) )
							{
								// Colision!! Update cell info:
								ptg->m_collisionGrid.updateCellInfo(
									ix,iy,
									k,							// The path (Alfa value)
									ptg->GetCPathPoint_d(k, n+1) // The collision distance
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
		ptg->SaveColGridsToFile( auxStr, robotShape );

	}	// "else" recompute all PTG

	MRPT_END
}



