/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/utils/CStream.h>
#include <mrpt/utils/round.h> // round()

#if HAVE_ALLOCA_H
# include <alloca.h>
#endif

using namespace mrpt;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;

/** Local stucture used in the next method (must be here for usage within STL stuff) */
struct TLocalPoint
{
	float x,y; int cx, cy;
};

/*---------------------------------------------------------------
					insertObservation

Insert the observation information into this map.
 ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::internal_insertObservation(
		const CObservation	*obs,
		const CPose3D			*robotPose)
{
// 	MRPT_START   // Avoid "try" since we use "alloca"

#define FRBITS	9

	CPose2D		robotPose2D;
	CPose3D		robotPose3D;

	// This is required to indicate the grid map has changed!
	//resetFeaturesCache();
	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	if (robotPose)
	{
		robotPose2D = CPose2D(*robotPose);
		robotPose3D = (*robotPose);
	}
	else
	{
		// Default values are (0,0,0)
	}

	if ( CLASS_ID(CObservation2DRangeScan)==obs->GetRuntimeClass())
	{
	/********************************************************************

				OBSERVATION TYPE: CObservation2DRangeScan

		********************************************************************/
		const CObservation2DRangeScan	*o = static_cast<const CObservation2DRangeScan*>( obs );
		CPose3D						sensorPose3D = robotPose3D + o->sensorPose;
		CPose2D						laserPose( sensorPose3D );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		bool		reallyInsert = o->isPlanarScan( insertionOptions.horizontalTolerance );
		unsigned int decimation = insertionOptions.decimation;

		// Check the altitude of the map (if feature enabled!)
		if ( insertionOptions.useMapAltitude &&
				fabs(insertionOptions.mapAltitude - sensorPose3D.z() ) > 0.001 )
		{
			reallyInsert = false;
		}

		// Manage horizontal scans, but with the sensor bottom-up:
		//  Use the z-axis direction of the transformed Z axis of the sensor coordinates:
		bool sensorIsBottomwards = sensorPose3D.getHomogeneousMatrixVal().get_unsafe(2,2) < 0;

		if ( reallyInsert )
		{
			// ---------------------------------------------
			//		Insert the scan as simple rays:
			// ---------------------------------------------
			int								cx,cy,N =  o->scan.size();
			float							px,py;
			double							A, dAK;

			// Parameters values:
			const float 	maxDistanceInsertion 	= insertionOptions.maxDistanceInsertion;
			const bool		invalidAsFree			= insertionOptions.considerInvalidRangesAsFreeSpace;
			float		new_x_max, new_x_min;
			float		new_y_max, new_y_min;
			float		last_valid_range	= maxDistanceInsertion;

			float		maxCertainty		= insertionOptions.maxOccupancyUpdateCertainty;
			cellType    logodd_observation  = p2l(maxCertainty);
			cellType    logodd_observation_occupied = 3*logodd_observation;

			// Assure minimum change in cells!
			if (logodd_observation<=0)
				logodd_observation=1;

			cellType    logodd_thres_occupied = OCCGRID_CELLTYPE_MIN+logodd_observation_occupied;
			cellType    logodd_thres_free     = OCCGRID_CELLTYPE_MAX-logodd_observation;


			int		K = updateInfoChangeOnly.enabled ? updateInfoChangeOnly.laserRaysSkip : decimation;
			size_t	idx,nRanges = o->scan.size();
			float	curRange=0;

			// Start position:
			px = laserPose.x();
			py = laserPose.y();

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			MRPT_CHECK_NORMAL_NUMBER(px);
			MRPT_CHECK_NORMAL_NUMBER(py);
#endif

			// Here we go! Now really insert changes in the grid:
			if ( !insertionOptions.wideningBeamsWithDistance )
			{
				// Method: Simple rays:
				// -------------------------------------

				// Reserve a temporary block of memory on the stack with "alloca": this memory has NOT to be deallocated,
				//  so it's ideal for an efficient, small buffer:
				float	*scanPoints_x = (float*) mrpt_alloca( sizeof(float) * nRanges );
				float	*scanPoints_y = (float*) mrpt_alloca( sizeof(float) * nRanges );

				float 	*scanPoint_x,*scanPoint_y;


				if (o->rightToLeft ^ sensorIsBottomwards )
				{
					A  = laserPose.phi() - 0.5 * o->aperture;
					dAK = K* o->aperture / N;
				}
				else
				{
					A  = laserPose.phi() + 0.5 * o->aperture;
					dAK = - K*o->aperture / N;
				}


				new_x_max = -(numeric_limits<float>::max)();
				new_x_min =  (numeric_limits<float>::max)();
				new_y_max = -(numeric_limits<float>::max)();
				new_y_min =  (numeric_limits<float>::max)();

				for (idx=0, scanPoint_x=scanPoints_x,scanPoint_y=scanPoints_y;idx<nRanges;idx+=K,scanPoint_x++,scanPoint_y++)
				{
					if ( o->validRange[idx] )
					{
						curRange = o->scan[idx];
						float R = min(maxDistanceInsertion,curRange);

						*scanPoint_x = px + cos(A)* R;
						*scanPoint_y = py + sin(A)* R;
						last_valid_range = curRange;
					}
					else
					{
						if (invalidAsFree)
						{
							// Invalid range:
							float R = min(maxDistanceInsertion,0.5f*last_valid_range);
							*scanPoint_x = px + cos(A)* R;
							*scanPoint_y = py + sin(A)* R;
						}
						else
						{
							*scanPoint_x = px;
							*scanPoint_y = py;
						}
					}
					A+=dAK;

					// Asjust size (will not change if not required):
					new_x_max = max( new_x_max, *scanPoint_x );
					new_x_min = min( new_x_min, *scanPoint_x );
					new_y_max = max( new_y_max, *scanPoint_y );
					new_y_min = min( new_y_min, *scanPoint_y );
				}

				// Add an extra margin:
				float securMargen = 15*resolution;

				if (new_x_max>x_max-securMargen)
						new_x_max+= 2*securMargen;
				else	new_x_max = x_max;
				if (new_x_min<x_min+securMargen)
						new_x_min-= 2;
				else	new_x_min = x_min;

				if (new_y_max>y_max-securMargen)
						new_y_max+= 2*securMargen;
				else	new_y_max = y_max;
				if (new_y_min<y_min+securMargen)
						new_y_min-= 2;
				else	new_y_min = y_min;

				// -----------------------
				//   Resize to make room:
				// -----------------------
				resizeGrid(new_x_min,new_x_max, new_y_min,new_y_max,0.5);

				// For updateCell_fast methods:
				cellType  *theMapArray = &map[0];
				unsigned  theMapSize_x = size_x;

				int  cx0 = x2idx(px);		// Remember: This must be after the resizeGrid!!
				int  cy0 = y2idx(py);


				// Insert rays:
				for (idx=0;idx<nRanges;idx+=K)
				{
					if ( !o->validRange[idx] && !invalidAsFree ) continue;

					// Starting position: Laser position
					cx = cx0;
					cy = cy0;

					// Target, in cell indexes:
					int trg_cx = x2idx(scanPoints_x[idx]);
					int trg_cy = y2idx(scanPoints_y[idx]);

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
					// The x> comparison implicitly holds if x<0
					ASSERT_( static_cast<unsigned int>(trg_cx)<size_x && static_cast<unsigned int>(trg_cy)<size_y );
	#endif

					// Use "fractional integers" to approximate float operations
					//  during the ray tracing:
					int Acx  = trg_cx - cx;
					int Acy  = trg_cy - cy;

					int Acx_ = abs(Acx);
					int Acy_ = abs(Acy);

					int nStepsRay = max( Acx_, Acy_ );
					if (!nStepsRay) continue; // May be...

					// Integers store "float values * 128"
					float  N_1 = 1.0f / nStepsRay;   // Avoid division twice.

					// Increments at each raytracing step:
					int  frAcx = round( (Acx<< FRBITS) * N_1 );  //  Acx*128 / N
					int  frAcy = round( (Acy<< FRBITS) * N_1 );  //  Acy*128 / N

					int frCX = cx << FRBITS;
					int frCY = cy << FRBITS;

					for (int nStep = 0;nStep<nStepsRay;nStep++)
					{
						updateCell_fast_free(cx,cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );

						frCX += frAcx;
						frCY += frAcy;

						cx = frCX >> FRBITS;
						cy = frCY >> FRBITS;
					}

					// And finally, the occupied cell at the end:
					// Only if:
					//  - It was a valid ray, and
					//  - The ray was not truncated
					if ( o->validRange[idx] && o->scan[idx]<maxDistanceInsertion )
						updateCell_fast_occupied(trg_cx,trg_cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );

				}  // End of each range

				mrpt_alloca_free( scanPoints_x );
				mrpt_alloca_free( scanPoints_y );

			}  // end insert with simple rays
			else
			{
				// ---------------------------------
				//  		Widen rays
				// Algorithm in: http://www.mrpt.org/Occupancy_Grids
				// ---------------------------------
				if (o->rightToLeft ^ sensorIsBottomwards )
				{
					A  = laserPose.phi() - 0.5 * o->aperture;
					dAK = K* o->aperture / N;
				}
				else
				{
					A  = laserPose.phi() + 0.5 * o->aperture;
					dAK = - K*o->aperture / N;
				}

				new_x_max = -(numeric_limits<float>::max)();
				new_x_min =  (numeric_limits<float>::max)();
				new_y_max = -(numeric_limits<float>::max)();
				new_y_min =  (numeric_limits<float>::max)();

				last_valid_range	= maxDistanceInsertion;
				for (idx=0;idx<nRanges;idx+=K)
				{
					float scanPoint_x,scanPoint_y;
					if ( o->validRange[idx] )
					{
						curRange = o->scan[idx];
						float R = min(maxDistanceInsertion,curRange);

						scanPoint_x = px + cos(A)* R;
						scanPoint_y = py + sin(A)* R;
						last_valid_range = curRange;
					}
					else
					{
						if (invalidAsFree)
						{
							// Invalid range:
							float R = min(maxDistanceInsertion,0.5f*last_valid_range);
							scanPoint_x = px + cos(A)* R;
							scanPoint_y = py + sin(A)* R;
						}
						else
						{
							scanPoint_x = px;
							scanPoint_y = py;
						}
					}
					A+=dAK;

					// Asjust size (will not change if not required):
					new_x_max = max( new_x_max, scanPoint_x );
					new_x_min = min( new_x_min, scanPoint_x );
					new_y_max = max( new_y_max, scanPoint_y );
					new_y_min = min( new_y_min, scanPoint_y );
				}

				// Add an extra margin:
				float securMargen = 15*resolution;

				if (new_x_max>x_max-securMargen)
						new_x_max+= 2*securMargen;
				else	new_x_max = x_max;
				if (new_x_min<x_min+securMargen)
						new_x_min-= 2;
				else	new_x_min = x_min;

				if (new_y_max>y_max-securMargen)
						new_y_max+= 2*securMargen;
				else	new_y_max = y_max;
				if (new_y_min<y_min+securMargen)
						new_y_min-= 2;
				else	new_y_min = y_min;

				// -----------------------
				//   Resize to make room:
				// -----------------------
				resizeGrid(new_x_min,new_x_max, new_y_min,new_y_max,0.5);

				// For updateCell_fast methods:
				cellType  *theMapArray = &map[0];
				unsigned  theMapSize_x = size_x;

				//int  cx0 = x2idx(px);		// Remember: This must be after the resizeGrid!!
				//int  cy0 = y2idx(py);


				// Now go and insert the triangles of each beam:
				// -----------------------------------------------
				if (o->rightToLeft ^ sensorIsBottomwards )
				{
					A  = laserPose.phi() - 0.5 * o->aperture;
					dAK = K* o->aperture / N;
				}
				else
				{
					A  = laserPose.phi() + 0.5 * o->aperture;
					dAK = - K*o->aperture / N;
				}

				// Insert the rays:
				// ------------------------------------------
				// Vertices of the triangle: In meters
				TLocalPoint P0,P1,P2, P1b;

				last_valid_range	= maxDistanceInsertion;

				const double dA_2 = 0.5 * o->aperture / N;
				for (idx=0;idx<nRanges; idx+=K, A+=dAK)
				{
					float	theR;		// The range of this beam
					if ( o->validRange[idx] )
					{
						curRange = o->scan[idx];
						last_valid_range = curRange;
						theR = min(maxDistanceInsertion,curRange);
					}
					else
					{
						// Invalid range:
						if (invalidAsFree)
						{
							theR = min(maxDistanceInsertion,0.5f*last_valid_range);
						}
						else continue; // Nothing to do
					}
					if (theR < resolution) continue; // Range must be larger than a cell...
					theR -= resolution;	// Remove one cell of length, which will be filled with "occupied" later.

					/* ---------------------------------------------------------
					      Fill one triangle with vertices: P0,P1,P2
					   --------------------------------------------------------- */
					P0.x = px;
					P0.y = py;

					P1.x = px + cos(A-dA_2) * theR;
					P1.y = py + sin(A-dA_2) * theR;

					P2.x = px + cos(A+dA_2) * theR;
					P2.y = py + sin(A+dA_2) * theR;

					// Order the vertices by the "y": P0->bottom, P2: top
					if (P2.y<P1.y) std::swap(P2,P1);
					if (P2.y<P0.y) std::swap(P2,P0);
					if (P1.y<P0.y) std::swap(P1,P0);


					// In cell indexes:
					P0.cx = x2idx( P0.x );	P0.cy = y2idx( P0.y );
					P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
					P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
					// The x> comparison implicitly holds if x<0
					ASSERT_( static_cast<unsigned int>(P0.cx)<size_x && static_cast<unsigned int>(P0.cy)<size_y );
					ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
					ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
	#endif

					struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:

					// Special case: one single row
					if (P0.cy==P2.cy && P0.cy==P1.cy)
					{
						// Optimized case:
						int min_cx = min3(P0.cx,P1.cx,P2.cx);
						int max_cx = max3(P0.cx,P1.cx,P2.cx);

						for (int ccx=min_cx;ccx<=max_cx;ccx++)
							updateCell_fast_free(ccx,P0.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
					}
					else
					{
						// The intersection point P1b in the segment P0-P2 at the "y" of P1:
						P1b.y = P1.y;
						P1b.x = P0.x + (P1.y-P0.y) * (P2.x-P0.x) / (P2.y-P0.y);

						P1b.cx= x2idx( P1b.x );	P1b.cy= y2idx( P1b.y );


						// Use "fractional integers" to approximate float operations during the ray tracing:
						// Integers store "float values * 128"
						const int Acx01 = P1.cx - P0.cx;
						const int Acy01 = P1.cy - P0.cy;
						const int Acx01b = P1b.cx - P0.cx;
						//const int Acy01b = P1b.cy - P0.cy;  // = Acy01

						// Increments at each raytracing step:
						const float inv_N_01 = 1.0f / ( max3(abs(Acx01),abs(Acy01),abs(Acx01b)) + 1 );	// Number of steps ^ -1
						const int  frAcx01 = round( (Acx01<< FRBITS) * inv_N_01 );  //  Acx*128 / N
						const int  frAcy01 = round( (Acy01<< FRBITS) * inv_N_01 );  //  Acy*128 / N
						const int  frAcx01b = round((Acx01b<< FRBITS)* inv_N_01 );  //  Acx*128 / N

						// ------------------------------------
						// First sub-triangle: P0-P1-P1b
						// ------------------------------------
						R1.cx  = P0.cx;
						R1.cy  = P0.cy;
						R1.frX = P0.cx << FRBITS;
						R1.frY = P0.cy << FRBITS;

						int frAx_R1=0, frAx_R2=0; //, frAy_R2;
						int frAy_R1 = frAcy01;

						// Start R1=R2 = P0... unlesss P0.cy == P1.cy, i.e. there is only one row:
						if (P0.cy!=P1.cy)
						{
							R2 = R1;
							//  R1 & R2 follow the edges: P0->P1  & P0->P1b
							//  R1 is forced to be at the left hand:
							if (P1.x<P1b.x)
							{
								// R1: P0->P1
								frAx_R1 = frAcx01;
								frAx_R2 = frAcx01b;
							}
							else
							{
								// R1: P0->P1b
								frAx_R1 = frAcx01b;
								frAx_R2 = frAcx01;
							}
						}
						else
						{
							R2.cx  = P1.cx;
							R2.cy  = P1.cy;
							R2.frX = P1.cx << FRBITS;
							//R2.frY = P1.cy << FRBITS;
						}

						int last_insert_cy = -1;
						//int last_insert_cx = -1;
						do
						{
							if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
							{
								last_insert_cy = R1.cy;
							//	last_insert_cx = R1.cx;

								for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
									updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
							}

							R1.frX += frAx_R1;    R1.frY += frAy_R1;
							R2.frX += frAx_R2;    // R1.frY += frAcy01;

							R1.cx = R1.frX >> FRBITS;
							R1.cy = R1.frY >> FRBITS;
							R2.cx = R2.frX >> FRBITS;
						} while ( R1.cy < P1.cy );

						// ------------------------------------
						// Second sub-triangle: P1-P1b-P2
						// ------------------------------------

						// Use "fractional integers" to approximate float operations during the ray tracing:
						// Integers store "float values * 128"
						const int Acx12  = P2.cx - P1.cx;
						const int Acy12  = P2.cy - P1.cy;
						const int Acx1b2 = P2.cx - P1b.cx;
						//const int Acy1b2 = Acy12

						// Increments at each raytracing step:
						const float inv_N_12 = 1.0f / ( max3(abs(Acx12),abs(Acy12),abs(Acx1b2)) + 1 );	// Number of steps ^ -1
						const int  frAcx12 = round( (Acx12<< FRBITS) * inv_N_12 );  //  Acx*128 / N
						const int  frAcy12 = round( (Acy12<< FRBITS) * inv_N_12 );  //  Acy*128 / N
						const int  frAcx1b2 = round((Acx1b2<< FRBITS)* inv_N_12 );  //  Acx*128 / N

						//struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:
						// R1, R2 follow edges P1->P2 & P1b->P2
						// R1 forced to be at the left hand
						frAy_R1 = frAcy12;
						if (!frAy_R1)
							frAy_R1 = 2 << FRBITS;	// If Ay=0, force it to be >0 so the "do...while" loop below ends in ONE iteration.

						if (P1.x<P1b.x)
						{
							// R1: P1->P2,  R2: P1b->P2
							R1.cx  = P1.cx;
							R1.cy  = P1.cy;
							R2.cx  = P1b.cx;
							R2.cy  = P1b.cy;
							frAx_R1 = frAcx12;
							frAx_R2 = frAcx1b2;
						}
						else
						{
							// R1: P1b->P2,  R2: P1->P2
							R1.cx  = P1b.cx;
							R1.cy  = P1b.cy;
							R2.cx  = P1.cx;
							R2.cy  = P1.cy;
							frAx_R1 = frAcx1b2;
							frAx_R2 = frAcx12;
						}

						R1.frX = R1.cx << FRBITS;
						R1.frY = R1.cy << FRBITS;
						R2.frX = R2.cx << FRBITS;
						R2.frY = R2.cy << FRBITS;

						last_insert_cy=-100;
						//last_insert_cx=-100;

						do
						{
							if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
							{
							//	last_insert_cx = R1.cx;
								last_insert_cy = R1.cy;
								for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
									updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
							}

							R1.frX += frAx_R1;    R1.frY += frAy_R1;
							R2.frX += frAx_R2;    // R1.frY += frAcy01;

							R1.cx = R1.frX >> FRBITS;
							R1.cy = R1.frY >> FRBITS;
							R2.cx = R2.frX >> FRBITS;
						} while ( R1.cy <= P2.cy );

					} // end of free-area normal case (not a single row)

					// ----------------------------------------------------
					// The final occupied cells along the edge P1<->P2
					// Only if:
					//  - It was a valid ray, and
					//  - The ray was not truncated
					// ----------------------------------------------------
					if ( o->validRange[idx] && o->scan[idx]<maxDistanceInsertion )
					{
						theR += resolution;

						P1.x = px + cos(A-dA_2) * theR;
						P1.y = py + sin(A-dA_2) * theR;

						P2.x = px + cos(A+dA_2) * theR;
						P2.y = py + sin(A+dA_2) * theR;

						P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
						P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

		#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
						// The x> comparison implicitly holds if x<0
						ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
						ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
		#endif

						// Special case: Only one cell:
						if (P2.cx==P1.cx && P2.cy==P1.cy)
						{
							updateCell_fast_occupied(P1.cx,P1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );
						}
						else
						{
							// Use "fractional integers" to approximate float operations during the ray tracing:
							// Integers store "float values * 128"
							const int AcxE  = P2.cx - P1.cx;
							const int AcyE  = P2.cy - P1.cy;

							// Increments at each raytracing step:
							const int nSteps = ( max(abs(AcxE),abs(AcyE)) + 1 );
							const float inv_N_12 = 1.0f / nSteps;	// Number of steps ^ -1
							const int  frAcxE = round( (AcxE<< FRBITS) * inv_N_12 );  //  Acx*128 / N
							const int  frAcyE = round( (AcyE<< FRBITS) * inv_N_12 );  //  Acy*128 / N

							R1.cx  = P1.cx;
							R1.cy  = P1.cy;
							R1.frX = R1.cx << FRBITS;
							R1.frY = R1.cy << FRBITS;

							for (int nStep=0;nStep<=nSteps;nStep++)
							{
								updateCell_fast_occupied(R1.cx,R1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );

								R1.frX += frAcxE;
								R1.frY += frAcyE;
								R1.cx = R1.frX >> FRBITS;
								R1.cy = R1.frY >> FRBITS;
							}

						} // end do a line

					} // end if we must set occupied cells

				}  // End of each range

			}  // end insert with beam widening

			// Finished:
			return true;
		}
		else
		{
			// A non-horizontal scan:
			return false;
		}
	}
	else if ( CLASS_ID(CObservationRange)==obs->GetRuntimeClass())
	{
		const CObservationRange *o = static_cast<const CObservationRange*>( obs );
		CPose3D spose;
		o->getSensorPose(spose);
		CPose3D						sensorPose3D = robotPose3D + spose;
		CPose2D						laserPose( sensorPose3D );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		bool		reallyInsert = true;
		unsigned int decimation = insertionOptions.decimation;

		// Check the altitude of the map (if feature enabled!)
		if ( insertionOptions.useMapAltitude &&
				fabs(insertionOptions.mapAltitude - sensorPose3D.z() ) > 0.001 )
		{
			reallyInsert = false;
		}
	    if ( reallyInsert )
		{
		    // ---------------------------------------------
			//		Insert the scan as simple rays:
			// ---------------------------------------------

			//int		/*cx,cy,*/ N =  o->sensedData.size();
			float	px,py;
			double	A, dAK;

			// Parameters values:
			const float 	maxDistanceInsertion 	= insertionOptions.maxDistanceInsertion;
			const bool		invalidAsFree			= insertionOptions.considerInvalidRangesAsFreeSpace;
			float		new_x_max, new_x_min;
			float		new_y_max, new_y_min;
			float		last_valid_range	= maxDistanceInsertion;

			float		maxCertainty		= insertionOptions.maxOccupancyUpdateCertainty;
			cellType    logodd_observation  = p2l(maxCertainty);
			cellType    logodd_observation_occupied = 3*logodd_observation;

			// Assure minimum change in cells!
			if (logodd_observation<=0)
				logodd_observation=1;

			cellType    logodd_thres_occupied = OCCGRID_CELLTYPE_MIN+logodd_observation_occupied;
			cellType    logodd_thres_free     = OCCGRID_CELLTYPE_MAX-logodd_observation;


			int		K = updateInfoChangeOnly.enabled ? updateInfoChangeOnly.laserRaysSkip : decimation;
			size_t	idx,nRanges = o->sensedData.size();
			float	curRange=0;

			// Start position:
			px = laserPose.x();
			py = laserPose.y();

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			MRPT_CHECK_NORMAL_NUMBER(px);
			MRPT_CHECK_NORMAL_NUMBER(py);
#endif
			// ---------------------------------
			//  		Widen rays
			// Algorithm in: http://www.mrpt.org/Occupancy_Grids
			// FIXME: doesn't support many different poses in one measurement
			// ---------------------------------
			A  = laserPose.phi();
			dAK = 0;

			new_x_max = -(numeric_limits<float>::max)();
			new_x_min =  (numeric_limits<float>::max)();
			new_y_max = -(numeric_limits<float>::max)();
			new_y_min =  (numeric_limits<float>::max)();

			last_valid_range	= maxDistanceInsertion;

			for (idx=0;idx<nRanges;idx+=K)
			{
				float scanPoint_x,scanPoint_y;
				if ( o->sensedData[idx].sensedDistance < maxDistanceInsertion )
				{
					curRange = o->sensedData[idx].sensedDistance;
					float R = min(maxDistanceInsertion,curRange);

					scanPoint_x = px + cos(A)* R;
					scanPoint_y = py + sin(A)* R;
					last_valid_range = curRange;
				}
				else
				{
					if (invalidAsFree)
					{
						// Invalid range:
						float R = min(maxDistanceInsertion,0.5f*last_valid_range);
						scanPoint_x = px + cos(A)* R;
						scanPoint_y = py + sin(A)* R;
					}
					else
					{
						scanPoint_x = px;
						scanPoint_y = py;
					}
				}
				A+=dAK;

				// Asjust size (will not change if not required):
				new_x_max = max( new_x_max, scanPoint_x );
				new_x_min = min( new_x_min, scanPoint_x );
				new_y_max = max( new_y_max, scanPoint_y );
				new_y_min = min( new_y_min, scanPoint_y );
			}

			// Add an extra margin:
			float securMargen = 15*resolution;

			if (new_x_max>x_max-securMargen)
					new_x_max+= 2*securMargen;
			else	new_x_max = x_max;
			if (new_x_min<x_min+securMargen)
					new_x_min-= 2;
			else	new_x_min = x_min;

			if (new_y_max>y_max-securMargen)
					new_y_max+= 2*securMargen;
			else	new_y_max = y_max;
			if (new_y_min<y_min+securMargen)
					new_y_min-= 2;
			else	new_y_min = y_min;

			// -----------------------
			//   Resize to make room:
			// -----------------------
			resizeGrid(new_x_min,new_x_max, new_y_min,new_y_max,0.5);

			// For updateCell_fast methods:
			cellType  *theMapArray = &map[0];
			unsigned  theMapSize_x = size_x;

			//int  cx0 = x2idx(px);		// Remember: This must be after the resizeGrid!!
			//int  cy0 = y2idx(py);


			// Now go and insert the triangles of each beam:
			// -----------------------------------------------
			A  = laserPose.phi() - 0.5 * o->sensorConeApperture;
			dAK = 0;

			// Insert the rays:
			// ------------------------------------------
			// Vertices of the triangle: In meters
			TLocalPoint P0,P1,P2, P1b;

			last_valid_range	= maxDistanceInsertion;

			const double dA_2 = 0.5 * o->sensorConeApperture;
			for (idx=0;idx<nRanges; idx+=K, A+=dAK)
			{
				float	theR;		// The range of this beam
				if ( o->sensedData[idx].sensedDistance < maxDistanceInsertion )
				{
					curRange = o->sensedData[idx].sensedDistance;
					last_valid_range = curRange;
					theR = min(maxDistanceInsertion,curRange);
				}
				else
				{
					// Invalid range:
					if (invalidAsFree)
					{
						theR = min(maxDistanceInsertion,0.5f*last_valid_range);
					}
					else continue; // Nothing to do
				}
				if (theR < resolution) continue; // Range must be larger than a cell...
				theR -= resolution;	// Remove one cell of length, which will be filled with "occupied" later.

				/* ---------------------------------------------------------
				      Fill one triangle with vertices: P0,P1,P2
				   --------------------------------------------------------- */
				P0.x = px;
				P0.y = py;

				P1.x = px + cos(A-dA_2) * theR;
				P1.y = py + sin(A-dA_2) * theR;

				P2.x = px + cos(A+dA_2) * theR;
				P2.y = py + sin(A+dA_2) * theR;

				// Order the vertices by the "y": P0->bottom, P2: top
				if (P2.y<P1.y) std::swap(P2,P1);
				if (P2.y<P0.y) std::swap(P2,P0);
				if (P1.y<P0.y) std::swap(P1,P0);


				// In cell indexes:
				P0.cx = x2idx( P0.x );	P0.cy = y2idx( P0.y );
				P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
				P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
				// The x> comparison implicitly holds if x<0
				ASSERT_( static_cast<unsigned int>(P0.cx)<size_x && static_cast<unsigned int>(P0.cy)<size_y );
				ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
				ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
#endif

				struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:

				// Special case: one single row
				if (P0.cy==P2.cy && P0.cy==P1.cy)
				{
					// Optimized case:
					int min_cx = min3(P0.cx,P1.cx,P2.cx);
					int max_cx = max3(P0.cx,P1.cx,P2.cx);

					for (int ccx=min_cx;ccx<=max_cx;ccx++)
						updateCell_fast_free(ccx,P0.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
				}
				else
				{
					// The intersection point P1b in the segment P0-P2 at the "y" of P1:
					P1b.y = P1.y;
					P1b.x = P0.x + (P1.y-P0.y) * (P2.x-P0.x) / (P2.y-P0.y);

					P1b.cx= x2idx( P1b.x );	P1b.cy= y2idx( P1b.y );


					// Use "fractional integers" to approximate float operations during the ray tracing:
					// Integers store "float values * 128"
					const int Acx01 = P1.cx - P0.cx;
					const int Acy01 = P1.cy - P0.cy;
					const int Acx01b = P1b.cx - P0.cx;
					//const int Acy01b = P1b.cy - P0.cy;  // = Acy01

					// Increments at each raytracing step:
					const float inv_N_01 = 1.0f / ( max3(abs(Acx01),abs(Acy01),abs(Acx01b)) + 1 );	// Number of steps ^ -1
					const int  frAcx01 = round( (Acx01<<FRBITS) * inv_N_01 );  //  Acx*128 / N
					const int  frAcy01 = round( (Acy01<<FRBITS) * inv_N_01 );  //  Acy*128 / N
					const int  frAcx01b = round((Acx01b<<FRBITS)* inv_N_01 );  //  Acx*128 / N

					// ------------------------------------
					// First sub-triangle: P0-P1-P1b
					// ------------------------------------
					R1.cx  = P0.cx;
					R1.cy  = P0.cy;
					R1.frX = P0.cx <<FRBITS;
					R1.frY = P0.cy <<FRBITS;

					int frAx_R1=0, frAx_R2=0; //, frAy_R2;
					int frAy_R1 = frAcy01;

					// Start R1=R2 = P0... unlesss P0.cy == P1.cy, i.e. there is only one row:
					if (P0.cy!=P1.cy)
					{
						R2 = R1;
						//  R1 & R2 follow the edges: P0->P1  & P0->P1b
						//  R1 is forced to be at the left hand:
						if (P1.x<P1b.x)
						{
							// R1: P0->P1
							frAx_R1 = frAcx01;
							frAx_R2 = frAcx01b;
						}
						else
						{
							// R1: P0->P1b
							frAx_R1 = frAcx01b;
							frAx_R2 = frAcx01;
						}
					}
					else
					{
						R2.cx  = P1.cx;
						R2.cy  = P1.cy;
						R2.frX = P1.cx <<FRBITS;
						//R2.frY = P1.cy <<FRBITS;
					}
					int last_insert_cy = -1;
					//int last_insert_cx = -1;
					do
					{
						if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
						{
							last_insert_cy = R1.cy;
						//	last_insert_cx = R1.cx;

							for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
								updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
						}

						R1.frX += frAx_R1;    R1.frY += frAy_R1;
						R2.frX += frAx_R2;    // R1.frY += frAcy01;

						R1.cx = R1.frX >> FRBITS;
						R1.cy = R1.frY >> FRBITS;
						R2.cx = R2.frX >> FRBITS;
					} while ( R1.cy < P1.cy );
					// ------------------------------------
					// Second sub-triangle: P1-P1b-P2
					// ------------------------------------

					// Use "fractional integers" to approximate float operations during the ray tracing:
					// Integers store "float values * 128"
					const int Acx12  = P2.cx - P1.cx;
					const int Acy12  = P2.cy - P1.cy;
					const int Acx1b2 = P2.cx - P1b.cx;
					//const int Acy1b2 = Acy12

					// Increments at each raytracing step:
					const float inv_N_12 = 1.0f / ( max3(abs(Acx12),abs(Acy12),abs(Acx1b2)) + 1 );	// Number of steps ^ -1
					const int  frAcx12 = round( (Acx12<<FRBITS) * inv_N_12 );  //  Acx*128 / N
					const int  frAcy12 = round( (Acy12<<FRBITS) * inv_N_12 );  //  Acy*128 / N
					const int  frAcx1b2 = round((Acx1b2<<FRBITS)* inv_N_12 );  //  Acx*128 / N

					//struct { int frX,frY; int cx,cy; } R1,R2;	// Fractional coords of the two rays:
					// R1, R2 follow edges P1->P2 & P1b->P2
					// R1 forced to be at the left hand
					frAy_R1 = frAcy12;
					if (!frAy_R1)
						frAy_R1 = 2 <<FRBITS;	// If Ay=0, force it to be >0 so the "do...while" loop below ends in ONE iteration.

					if (P1.x<P1b.x)
					{
						// R1: P1->P2,  R2: P1b->P2
						R1.cx  = P1.cx;
						R1.cy  = P1.cy;
						R2.cx  = P1b.cx;
						R2.cy  = P1b.cy;
						frAx_R1 = frAcx12;
						frAx_R2 = frAcx1b2;
					}
					else
					{
						// R1: P1b->P2,  R2: P1->P2
						R1.cx  = P1b.cx;
						R1.cy  = P1b.cy;
						R2.cx  = P1.cx;
						R2.cy  = P1.cy;
						frAx_R1 = frAcx1b2;
						frAx_R2 = frAcx12;
					}

					R1.frX = R1.cx <<FRBITS;
					R1.frY = R1.cy <<FRBITS;
					R2.frX = R2.cx <<FRBITS;
					R2.frY = R2.cy <<FRBITS;

					last_insert_cy=-100;
					//last_insert_cx=-100;

					do
					{
						if (last_insert_cy!=R1.cy) // || last_insert_cx!=R1.cx)
						{
						//	last_insert_cx = R1.cx;
							last_insert_cy = R1.cy;
							for (int ccx=R1.cx;ccx<=R2.cx;ccx++)
								updateCell_fast_free(ccx,R1.cy, logodd_observation, logodd_thres_free, theMapArray, theMapSize_x );
						}

						R1.frX += frAx_R1;    R1.frY += frAy_R1;
						R2.frX += frAx_R2;    // R1.frY += frAcy01;

						R1.cx = R1.frX >> FRBITS;
						R1.cy = R1.frY >> FRBITS;
						R2.cx = R2.frX >> FRBITS;
					} while ( R1.cy <= P2.cy );

				} // end of free-area normal case (not a single row)

				// ----------------------------------------------------
				// The final occupied cells along the edge P1<->P2
				// Only if:
				//  - It was a valid ray, and
				//  - The ray was not truncated
				// ----------------------------------------------------
				if ( o->sensedData[idx].sensedDistance < maxDistanceInsertion )
				{
					theR += resolution;

					P1.x = px + cos(A-dA_2) * theR;
					P1.y = py + sin(A-dA_2) * theR;

					P2.x = px + cos(A+dA_2) * theR;
					P2.y = py + sin(A+dA_2) * theR;

					P1.cx = x2idx( P1.x );	P1.cy = y2idx( P1.y );
					P2.cx = x2idx( P2.x );	P2.cy = y2idx( P2.y );

	#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
					// The x> comparison implicitly holds if x<0
					ASSERT_( static_cast<unsigned int>(P1.cx)<size_x && static_cast<unsigned int>(P1.cy)<size_y );
					ASSERT_( static_cast<unsigned int>(P2.cx)<size_x && static_cast<unsigned int>(P2.cy)<size_y );
	#endif

					// Special case: Only one cell:
					if (P2.cx==P1.cx && P2.cy==P1.cy)
					{
						updateCell_fast_occupied(P1.cx,P1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );
					}
					else
					{
						// Use "fractional integers" to approximate float operations during the ray tracing:
						// Integers store "float values * 128"
						const int AcxE  = P2.cx - P1.cx;
						const int AcyE  = P2.cy - P1.cy;

						// Increments at each raytracing step:
						const int nSteps = ( max(abs(AcxE),abs(AcyE)) + 1 );
						const float inv_N_12 = 1.0f / nSteps;	// Number of steps ^ -1
						const int  frAcxE = round( (AcxE<<FRBITS) * inv_N_12 );  //  Acx*128 / N
						const int  frAcyE = round( (AcyE<<FRBITS) * inv_N_12 );  //  Acy*128 / N

						R1.cx  = P1.cx;
						R1.cy  = P1.cy;
						R1.frX = R1.cx <<FRBITS;
						R1.frY = R1.cy <<FRBITS;

						for (int nStep=0;nStep<=nSteps;nStep++)
						{
							updateCell_fast_occupied(R1.cx,R1.cy, logodd_observation_occupied, logodd_thres_occupied, theMapArray, theMapSize_x );

							R1.frX += frAcxE;
							R1.frY += frAcyE;
							R1.cx = R1.frX >> FRBITS;
							R1.cy = R1.frY >> FRBITS;
						}

					} // end do a line

				} // end if we must set occupied cells

			}  // End of each range

			return true;
		} // end reallyInsert
		else
		    return false;
	}
	else
	{
		/********************************************************************
				OBSERVATION TYPE: Unknown
		********************************************************************/
		return false;
	}

//	MRPT_END
}


/*---------------------------------------------------------------
	Initilization of values, don't needed to be called directly.
  ---------------------------------------------------------------*/
COccupancyGridMap2D::TInsertionOptions::TInsertionOptions() :
	mapAltitude							( 0 ),
	useMapAltitude						( false ),
	maxDistanceInsertion				(  15.0f ),
	maxOccupancyUpdateCertainty			(  0.65f ),
	considerInvalidRangesAsFreeSpace	(  true ),
	decimation							( 1 ),
	horizontalTolerance					( DEG2RAD(0.05) ),

	CFD_features_gaussian_size			( 1 ),
	CFD_features_median_size			( 3 ),

	wideningBeamsWithDistance			( false )
{
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR(mapAltitude,float,  					iniFile, section );
	MRPT_LOAD_CONFIG_VAR(maxDistanceInsertion,float,  			iniFile, section );
	MRPT_LOAD_CONFIG_VAR(maxOccupancyUpdateCertainty,float,  	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(useMapAltitude,bool,  					iniFile, section );
	MRPT_LOAD_CONFIG_VAR(considerInvalidRangesAsFreeSpace,bool,	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(decimation,int,  						iniFile, section );
	MRPT_LOAD_CONFIG_VAR_DEGREES(horizontalTolerance, 		 	iniFile, section );

	MRPT_LOAD_CONFIG_VAR(CFD_features_gaussian_size,float,  	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(CFD_features_median_size,float,  	iniFile, section );
	MRPT_LOAD_CONFIG_VAR(wideningBeamsWithDistance,bool,  	iniFile, section );
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [COccupancyGridMap2D::TInsertionOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(mapAltitude, float)
	LOADABLEOPTS_DUMP_VAR(maxDistanceInsertion, float)
	LOADABLEOPTS_DUMP_VAR(maxOccupancyUpdateCertainty, float)
	LOADABLEOPTS_DUMP_VAR(useMapAltitude, bool)
	LOADABLEOPTS_DUMP_VAR(considerInvalidRangesAsFreeSpace, bool)
	LOADABLEOPTS_DUMP_VAR(decimation, int)
	LOADABLEOPTS_DUMP_VAR(horizontalTolerance, float)
	LOADABLEOPTS_DUMP_VAR(CFD_features_gaussian_size, float)
	LOADABLEOPTS_DUMP_VAR(CFD_features_median_size, float)
	LOADABLEOPTS_DUMP_VAR(wideningBeamsWithDistance, bool)

	out.printf("\n");
}


void COccupancyGridMap2D::OnPostSuccesfulInsertObs(const mrpt::obs::CObservation *)
{
	m_is_empty = false;
}
