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
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/utils/CStream.h>


using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;



/*---------------------------------------------------------------
 Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
	takenFrom The robot's pose the observation is supposed to be taken from.
	obs The observation.
 This method returns a likelihood in the range [0,1].
 ---------------------------------------------------------------*/
double	 COccupancyGridMap2D::internal_computeObservationLikelihood(
			const CObservation		*obs,
			const CPose3D			&takenFrom3D )
{
	// Ignore laser scans if they are not planar or they are not
	//  at the altitude of this grid map:
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		const CObservation2DRangeScan	*scan = static_cast<const CObservation2DRangeScan*>(obs);
		if (!scan->isPlanarScan(insertionOptions.horizontalTolerance))
			return -10;
		if (insertionOptions.useMapAltitude &&
			fabs(insertionOptions.mapAltitude - scan->sensorPose.z() ) > 0.01 )
			return -10;

		// OK, go on...
	}

	// Execute according to the selected method:
	// --------------------------------------------
	CPose2D   takenFrom = CPose2D(takenFrom3D);  // 3D -> 2D, we are in a gridmap...

	switch (likelihoodOptions.likelihoodMethod)
	{
	default:
	case lmRayTracing:
		return computeObservationLikelihood_rayTracing(obs,takenFrom);

	case lmMeanInformation:
		return computeObservationLikelihood_MI(obs,takenFrom);

	case lmConsensus:
		return computeObservationLikelihood_Consensus(obs,takenFrom);

	case lmCellsDifference:
		return computeObservationLikelihood_CellsDifference(obs,takenFrom);

	case lmLikelihoodField_Thrun:
		return computeObservationLikelihood_likelihoodField_Thrun(obs,takenFrom);

	case lmLikelihoodField_II:
		return computeObservationLikelihood_likelihoodField_II(obs,takenFrom);

	case lmConsensusOWA:
		return computeObservationLikelihood_ConsensusOWA(obs,takenFrom);
	};

}

/*---------------------------------------------------------------
			computeObservationLikelihood_Consensus
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_Consensus(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	double		likResult = 0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( obs->GetRuntimeClass() != CLASS_ID(CObservation2DRangeScan) )
	{
		//THROW_EXCEPTION("This method is defined for 'CObservation2DRangeScan' classes only.");
		return 1e-3;
	}
	// Observation is a laser range scan:
	// -------------------------------------------
	const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

	// Insert only HORIZONTAL scans, since the grid is supposed to
	//  be a horizontal representation of space.
	if ( ! o->isPlanarScan(insertionOptions.horizontalTolerance) ) return 0.5f;		// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

	// Assure we have a 2D points-map representation of the points from the scan:
	const CPointsMap *compareMap = o->buildAuxPointsMap<mrpt::maps::CPointsMap>();

	// Observation is a points map:
	// -------------------------------------------
	size_t			Denom=0;
//	int			Acells = 1;
	TPoint2D pointGlobal,pointLocal;


	// Get the points buffers:

	//	compareMap.getPointsBuffer( n, xs, ys, zs );
	const size_t n = compareMap->size();

	for (size_t i=0;i<n;i+=likelihoodOptions.consensus_takeEachRange)
	{
		// Get the point and pass it to global coordinates:
		compareMap->getPoint(i,pointLocal);
		takenFrom.composePoint(pointLocal, pointGlobal);

		int		cx0 = x2idx( pointGlobal.x );
		int		cy0 = y2idx( pointGlobal.y );

		likResult += 1-getCell_nocheck(cx0,cy0);
		Denom++;
	}
	if (Denom)	likResult/=Denom;
	likResult = pow(likResult, static_cast<double>( likelihoodOptions.consensus_pow ) );

	return log(likResult);
}

/*---------------------------------------------------------------
			computeObservationLikelihood_ConsensusOWA
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_ConsensusOWA(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	double		likResult = 0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		//THROW_EXCEPTION("This method is defined for 'CObservation2DRangeScan' classes only.");
		return 1e-3;
	}
	// Observation is a laser range scan:
	// -------------------------------------------
	const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

	// Insert only HORIZONTAL scans, since the grid is supposed to
	//  be a horizontal representation of space.
	if ( ! o->isPlanarScan(insertionOptions.horizontalTolerance) ) return 0.5;		// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

	// Assure we have a 2D points-map representation of the points from the scan:
	CPointsMap::TInsertionOptions	insOpt;
	insOpt.minDistBetweenLaserPoints	= -1;		// ALL the laser points

	const CPointsMap *compareMap = o->buildAuxPointsMap<mrpt::maps::CPointsMap>( &insOpt );

	// Observation is a points map:
	// -------------------------------------------
	int				Acells = 1;
	TPoint2D		pointGlobal,pointLocal;

	// Get the points buffers:
	const size_t n = compareMap->size();

	// Store the likelihood values in this vector:
	likelihoodOutputs.OWA_pairList.clear();
	for (size_t i=0;i<n;i++)
	{
		// Get the point and pass it to global coordinates:
		compareMap->getPoint(i,pointLocal);
		takenFrom.composePoint(pointLocal, pointGlobal);

		int		cx0 = x2idx( pointGlobal.x );
		int		cy0 = y2idx( pointGlobal.y );

		int		cxMin = max(0,cx0 - Acells);
		int		cxMax = min(static_cast<int>(size_x)-1,cx0 + Acells);
		int		cyMin = max(0,cy0 - Acells);
		int		cyMax = min(static_cast<int>(size_y)-1,cy0 + Acells);

		double	lik = 0;

		for (int cx=cxMin;cx<=cxMax;cx++)
			for (int cy=cyMin;cy<=cyMax;cy++)
				lik += 1-getCell_nocheck(cx,cy);

		int		nCells = (cxMax-cxMin+1)*(cyMax-cyMin+1);
		ASSERT_(nCells>0);
		lik/=nCells;

		TPairLikelihoodIndex	element;
		element.first = lik;
		element.second = pointGlobal;
		likelihoodOutputs.OWA_pairList.push_back( element );
	} // for each range point

	// Sort the list of likelihood values, in descending order:
	// ------------------------------------------------------------
	std::sort(likelihoodOutputs.OWA_pairList.begin(),likelihoodOutputs.OWA_pairList.end());

	// Cut the vector to the highest "likelihoodOutputs.OWA_length" elements:
	size_t	M = likelihoodOptions.OWA_weights.size();
	ASSERT_( likelihoodOutputs.OWA_pairList.size()>=M );

	likelihoodOutputs.OWA_pairList.resize(M);
	likelihoodOutputs.OWA_individualLikValues.resize( M );
	likResult = 0;
	for (size_t k=0;k<M;k++)
	{
		likelihoodOutputs.OWA_individualLikValues[k] = likelihoodOutputs.OWA_pairList[k].first;
		likResult+= likelihoodOptions.OWA_weights[k] * likelihoodOutputs.OWA_individualLikValues[k];
	}

	return log(likResult);
}

/*---------------------------------------------------------------
			computeObservationLikelihood_CellsDifference
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_CellsDifference(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	 double		ret = 0.5;

	 // This function depends on the observation type:
	 // -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	 {
		 // Observation is a laser range scan:
		 // -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return 0.5;	// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

	 // Build a copy of this occupancy grid:
		COccupancyGridMap2D		compareGrid(takenFrom.x()-10,takenFrom.x()+10,takenFrom.y()-10,takenFrom.y()+10,resolution);
		CPose3D					robotPose(takenFrom);
		int						Ax, Ay;

		// Insert in this temporary grid:
		compareGrid.insertionOptions.maxDistanceInsertion			= insertionOptions.maxDistanceInsertion;
		compareGrid.insertionOptions.maxOccupancyUpdateCertainty	= 0.95f;
		o->insertObservationInto( &compareGrid, &robotPose );

		// Save Cells offset between the two grids:
		Ax = round((x_min - compareGrid.x_min) / resolution);
		Ay = round((y_min - compareGrid.y_min) / resolution);

		int			nCellsCompared = 0;
		float		cellsDifference = 0;
		int			x0 = max(0,Ax);
		int			y0 = max(0,Ay);
		int			x1 = min(compareGrid.size_x, size_x+Ax);
		int			y1 = min(compareGrid.size_y, size_y+Ay);

		for (int x=x0;x<x1;x+=1)
		{
			for (int y=y0;y<y1;y+=1)
			{
				float	xx = compareGrid.idx2x(x);
				float	yy = compareGrid.idx2y(y);

				float	c1 = getPos(xx,yy);
				float	c2 = compareGrid.getCell(x,y);
				if ( c2<0.45f || c2>0.55f )
				{
					nCellsCompared++;
					if ((c1>0.5 && c2<0.5) || (c1<0.5 && c2>0.5))
						cellsDifference++;
				}
			}
		}
		ret = 1 - cellsDifference / (nCellsCompared);
	 }
	 return log(ret);
}

/*---------------------------------------------------------------
			computeObservationLikelihood_MI
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_MI(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	MRPT_START

 	CPose3D			poseRobot(takenFrom);
	double			res;

	// Dont modify the grid, only count the changes in Information
	updateInfoChangeOnly.enabled = true;
	insertionOptions.maxDistanceInsertion*= likelihoodOptions.MI_ratio_max_distance;

	// Reset the new information counters:
	updateInfoChangeOnly.cellsUpdated = 0;
	updateInfoChangeOnly.I_change = 0;
	updateInfoChangeOnly.laserRaysSkip = likelihoodOptions.MI_skip_rays;

	// Insert the observation (It will not be really inserted, only the information counted)
	insertObservation(obs,&poseRobot);

	// Compute the change in I aported by the observation:
	double	newObservation_mean_I;
	if (updateInfoChangeOnly.cellsUpdated)
			newObservation_mean_I = updateInfoChangeOnly.I_change / updateInfoChangeOnly.cellsUpdated;
	else	newObservation_mean_I = 0;

	// Let the normal mode enabled, i.e. the grid can be updated
	updateInfoChangeOnly.enabled = false;
	insertionOptions.maxDistanceInsertion/=likelihoodOptions.MI_ratio_max_distance;


	res = pow(newObservation_mean_I, static_cast<double>(likelihoodOptions.MI_exponent) );

	return log(res);

	MRPT_END
 }

double	 COccupancyGridMap2D::computeObservationLikelihood_rayTracing(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	 double		ret=0;

	 // This function depends on the observation type:
	 // -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	 {
		 // Observation is a laser range scan:
		 // -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );
		 CObservation2DRangeScan		simulatedObs;

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return 0.5;	// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

		 // The number of simulated rays will be original range scan rays / DOWNRATIO
		 int		decimation = likelihoodOptions.rayTracing_decimation;
		 int		nRays     = o->scan.size();

		 // Perform simulation using same parameters than real observation:
		 simulatedObs.aperture = o->aperture;
		 simulatedObs.maxRange = o->maxRange;
		 simulatedObs.rightToLeft = o->rightToLeft;
		 simulatedObs.sensorPose = o->sensorPose;

		 // Performs the scan simulation:
		 laserScanSimulator(
			simulatedObs,		// The in/out observation
			takenFrom,			// robot pose
			0.45f,				// Cells threshold
			nRays,				// Scan length
			0,
			decimation	);

		 /** /
		 {
			FILE	*f;

			f=os::fopen("scan_sim.txt","wt");
			for (int i=0;i<nRays;i++) os::fprintf(f,"%f ",simulatedObs.validRange[i] ? simulatedObs.scan[i]:0);
			os::fclose(f);
			f=os::fopen("scan_meas.txt","wt");
			for (i=0;i<nRays;i++) os::fprintf(f,"%f ",o->validRange[i] ? o->scan[i]:0);
			os::fclose(f);
		 }
		 / **/

		 double		stdLaser   = likelihoodOptions.rayTracing_stdHit;
		 double		stdSqrt2 = sqrt(2.0f) * stdLaser;

		 // Compute likelihoods:
		 ret = 1;
		 //bool		useDF = likelihoodOptions.rayTracing_useDistanceFilter;
		 float		r_sim,r_obs;
		 double		likelihood;

		 for (int j=0;j<nRays;j+=decimation)
		 {
			// Simulated and measured ranges:
			r_sim = simulatedObs.scan[j];
			r_obs = o->scan[ j ];

			// Is a valid range?
			if ( o->validRange[j] )
			{
				likelihood = 0.1/o->maxRange + 0.9*exp( -square( min((float)fabs(r_sim-r_obs),2.0f)/stdSqrt2) );
				ret += log(likelihood);
				//printf("Sim=%f\tReal=%f\tlik=%f\n",r_sim,r_obs,likelihood);
			}

		 }
	 }

	 return ret;
}
/**/

/*---------------------------------------------------------------
			computeObservationLikelihood_likelihoodField_Thrun
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_likelihoodField_Thrun(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	MRPT_START

	double		ret=0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( IS_CLASS(obs, CObservation2DRangeScan) )
	{
		// Observation is a laser range scan:
		// -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return -10;

		// Assure we have a 2D points-map representation of the points from the scan:
		CPointsMap::TInsertionOptions		opts;
		opts.minDistBetweenLaserPoints	= resolution*0.5f;
		opts.isPlanarMap				= true; // Already filtered above!
		opts.horizontalTolerance		= insertionOptions.horizontalTolerance;

		// Compute the likelihood of the points in this grid map:
		ret = computeLikelihoodField_Thrun( o->buildAuxPointsMap<mrpt::maps::CPointsMap>(&opts), &takenFrom );

	} // end of observation is a scan range 2D
	else if ( IS_CLASS(obs, CObservationRange) )
	{
	    // Sonar-like observations:
	    // ---------------------------------------
		const CObservationRange		*o = static_cast<const CObservationRange*>( obs );

        // Create a point map representation of the observation:
	    CSimplePointsMap pts;
	    pts.insertionOptions.minDistBetweenLaserPoints	= resolution*0.5f;
	    pts.insertObservation(o);

		// Compute the likelihood of the points in this grid map:
		ret = computeLikelihoodField_Thrun( &pts, &takenFrom );
	}

	return ret;

	MRPT_END

}

/*---------------------------------------------------------------
		computeObservationLikelihood_likelihoodField_II
---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeObservationLikelihood_likelihoodField_II(
			const CObservation		*obs,
			const CPose2D				&takenFrom )
{
	MRPT_START

	double		ret=0;

	// This function depends on the observation type:
	// -----------------------------------------------------
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		// Observation is a laser range scan:
		// -------------------------------------------
		const CObservation2DRangeScan		*o = static_cast<const CObservation2DRangeScan*>( obs );

		// Insert only HORIZONTAL scans, since the grid is supposed to
		//  be a horizontal representation of space.
		if (!o->isPlanarScan(insertionOptions.horizontalTolerance)) return 0.5f;	// NO WAY TO ESTIMATE NON HORIZONTAL SCANS!!

		// Assure we have a 2D points-map representation of the points from the scan:

		// Compute the likelihood of the points in this grid map:
		ret = computeLikelihoodField_II( o->buildAuxPointsMap<mrpt::maps::CPointsMap>(), &takenFrom );

	} // end of observation is a scan range 2D

	return ret;

	MRPT_END

}


/*---------------------------------------------------------------
					computeLikelihoodField_Thrun
 ---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeLikelihoodField_Thrun( const CPointsMap	*pm, const CPose2D *relativePose )
{
	MRPT_START

	double		ret;
	size_t		N = pm->size();
	int		K = (int)ceil(likelihoodOptions.LF_maxCorrsDistance/*m*/ / resolution);	// The size of the checking area for matchings:

	bool		Product_T_OrSum_F = !likelihoodOptions.LF_alternateAverageMethod;

	if (!N)
	{
		return -100; // No way to estimate this likelihood!!
	}

	// Compute the likelihoods for each point:
	ret = 0;

	float		stdHit	= likelihoodOptions.LF_stdHit;
	float		zHit	= likelihoodOptions.LF_zHit;
	float		zRandom	= likelihoodOptions.LF_zRandom;
	float		zRandomMaxRange	= likelihoodOptions.LF_maxRange;
	float		zRandomTerm = zRandom / zRandomMaxRange;
	float		Q = -0.5f / square(stdHit);
	int			M = 0;

	unsigned int	size_x_1 = size_x-1;
	unsigned int	size_y_1 = size_y-1;

	// Aux. variables for the "for j" loop:
	double		thisLik;
	double		maxCorrDist_sq = square(likelihoodOptions.LF_maxCorrsDistance);
	double		minimumLik = zRandomTerm  + zHit * exp( Q * maxCorrDist_sq );
	double		ccos,ssin;
	float		occupiedMinDist;

#define LIK_LF_CACHE_INVALID    (66)

    if (likelihoodOptions.enableLikelihoodCache)
    {
        // Reset the precomputed likelihood values map
        if (precomputedLikelihoodToBeRecomputed)
        {
			if (!map.empty())
					precomputedLikelihood.assign( map.size(),LIK_LF_CACHE_INVALID);
			else	precomputedLikelihood.clear();

			precomputedLikelihoodToBeRecomputed = false;
        }
    }

	cellType	thresholdCellValue = p2l(0.5f);
	int			decimation = likelihoodOptions.LF_decimation;

	const double _resolution = this->resolution;
	const double constDist2DiscrUnits = 100 / (_resolution * _resolution);
	const double constDist2DiscrUnits_INV = 1.0 / constDist2DiscrUnits;


	if (N<10) decimation = 1;

	TPoint2D	pointLocal;
	TPoint2D	pointGlobal;

	for (size_t j=0;j<N;j+= decimation)
	{

		occupiedMinDist = maxCorrDist_sq; // The max.distance

		// Get the point and pass it to global coordinates:
		if (relativePose)
		{
			pm->getPoint(j,pointLocal);
			//pointGlobal = *relativePose + pointLocal;
#ifdef HAVE_SINCOS
			::sincos(relativePose->phi(), &ssin,&ccos);
#else
			ccos = cos(relativePose->phi());
			ssin = sin(relativePose->phi());
#endif
			pointGlobal.x = relativePose->x() + pointLocal.x * ccos - pointLocal.y * ssin;
			pointGlobal.y = relativePose->y() + pointLocal.x * ssin + pointLocal.y * ccos;
		}
		else
		{
			pm->getPoint(j,pointGlobal);
		}

		// Point to cell indixes
		int cx = x2idx( pointGlobal.x );
		int cy = y2idx( pointGlobal.y );

		// Precomputed table:
		// Tip: Comparison cx<0 is implicit in (unsigned)(x)>size...
		if ( static_cast<unsigned>(cx)>=size_x_1 || static_cast<unsigned>(cy)>=size_y_1 )
		{
			// We are outside of the map: Assign the likelihood for the max. correspondence distance:
			thisLik = minimumLik;
		}
		else
		{
			// We are into the map limits:
            if (likelihoodOptions.enableLikelihoodCache)
            {
                thisLik = precomputedLikelihood[ cx+cy*size_x ];
            }

			if (!likelihoodOptions.enableLikelihoodCache || thisLik==LIK_LF_CACHE_INVALID )
			{
				// Compute now:
				// -------------
				// Find the closest occupied cell in a certain range, given by K:
				int xx1 = max(0,cx-K);
				int xx2 = min(size_x_1,(unsigned)(cx+K));
				int yy1 = max(0,cy-K);
				int yy2 = min(size_y_1,(unsigned)(cy+K));

				// Optimized code: this part will be invoked a *lot* of times:
				{
					cellType  *mapPtr  = &map[xx1+yy1*size_x]; // Initial pointer position
					unsigned   incrAfterRow = size_x - ((xx2-xx1)+1);

					signed int Ax0 = 10*(xx1-cx);
					signed int Ay  = 10*(yy1-cy);

					unsigned int occupiedMinDistInt = mrpt::utils::round( maxCorrDist_sq * constDist2DiscrUnits );

					for (int yy=yy1;yy<=yy2;yy++)
					{
						unsigned int Ay2 = square((unsigned int)(Ay)); // Square is faster with unsigned.
						signed short Ax=Ax0;
						cellType  cell;

						for (int xx=xx1;xx<=xx2;xx++)
						{
							if ( (cell =*mapPtr++) < thresholdCellValue )
							{
								unsigned int d = square((unsigned int)(Ax)) + Ay2;
								keep_min(occupiedMinDistInt, d);
							}
							Ax += 10;
						}
						// Go to (xx1,yy++)
						mapPtr += incrAfterRow;
						Ay += 10;
					}

					occupiedMinDist = occupiedMinDistInt * constDist2DiscrUnits_INV ;
				}

				if (likelihoodOptions.LF_useSquareDist)
					occupiedMinDist*=occupiedMinDist;

				thisLik = zRandomTerm  + zHit * exp( Q * occupiedMinDist );

                if (likelihoodOptions.enableLikelihoodCache)
                    // And save it into the table and into "thisLik":
                    precomputedLikelihood[ cx+cy*size_x ] = thisLik;
			}
		}

		// Update the likelihood:
		if (Product_T_OrSum_F)
		{
			ret += log(thisLik);
		}
		else
		{
			ret += thisLik;
			M++;
		}
	} // end of for each point in the scan

	if (!Product_T_OrSum_F)
		ret = log( ret / M );

	return ret;

	MRPT_END
}

/*---------------------------------------------------------------
					computeLikelihoodField_II
 ---------------------------------------------------------------*/
double	 COccupancyGridMap2D::computeLikelihoodField_II( const CPointsMap	*pm, const CPose2D *relativePose )
{
	MRPT_START

	double		ret;
	size_t		N = pm->size();

	if (!N) return 1e-100; // No way to estimate this likelihood!!

	// Compute the likelihoods for each point:
	ret = 0;
//	if (likelihoodOptions.LF_alternateAverageMethod)
//			ret = 0;
//	else	ret = 1;

	TPoint2D	pointLocal,pointGlobal;

	float		zRandomTerm = 1.0f / likelihoodOptions.LF_maxRange;
	float		Q = -0.5f / square( likelihoodOptions.LF_stdHit );

	// Aux. cell indixes variables:
	int			cx,cy;
	size_t		j;
	int			cx0,cy0;
	int			cx_min, cx_max;
	int			cy_min, cy_max;
	int			maxRangeInCells = (int)ceil(likelihoodOptions.LF_maxCorrsDistance / resolution);
	int			nCells = 0;

	// -----------------------------------------------------
	// Compute around a window of neigbors around each point
	// -----------------------------------------------------
	for (j=0;j<N;j+= likelihoodOptions.LF_decimation)
	{
		// Get the point and pass it to global coordinates:
		// ---------------------------------------------
		if (relativePose)
		{
			pm->getPoint(j,pointLocal);
			pointGlobal = *relativePose + pointLocal;
		}
		else
		{
			pm->getPoint(j,pointGlobal);
		}

		// Point to cell indixes:
		// ---------------------------------------------
		cx0 = x2idx( pointGlobal.x );
		cy0 = y2idx( pointGlobal.y );

		// Compute the range of cells to compute:
		// ---------------------------------------------
		cx_min = max( cx0-maxRangeInCells,0);
		cx_max = min( cx0+maxRangeInCells,static_cast<int>(size_x));
		cy_min = max( cy0-maxRangeInCells,0);
		cy_max = min( cy0+maxRangeInCells,static_cast<int>(size_y));

//		debugImg.rectangle(cx_min,cy_min,cx_max,cy_max,0xFF0000 );

		// Compute over the window of cells:
		// ---------------------------------------------
		double  lik = 0;
		for (cx=cx_min;cx<=cx_max;cx++)
		{
			for (cy=cy_min;cy<=cy_max;cy++)
			{
				float	P_free = getCell(cx,cy);
				float	termDist = exp(Q*(square(idx2x(cx)-pointGlobal.x)+square(idx2y(cy)-pointGlobal.y) ));

				lik += P_free	  * zRandomTerm +
					   (1-P_free) * termDist;
			} // end for cy
		} // end for cx

		// Update the likelihood:
		if (likelihoodOptions.LF_alternateAverageMethod)
				ret += lik;
		else	ret += log(lik/((cy_max-cy_min+1)*(cx_max-cx_min+1)));
		nCells++;

	} // end of for each point in the scan

	if (likelihoodOptions.LF_alternateAverageMethod && nCells>0)
			ret = log(ret/nCells);
	else	ret = ret/nCells;

	return ret;

	MRPT_END
}



/*---------------------------------------------------------------
	Initilization of values, don't needed to be called directly.
  ---------------------------------------------------------------*/
COccupancyGridMap2D::TLikelihoodOptions::TLikelihoodOptions() :
	likelihoodMethod				( lmLikelihoodField_Thrun),

	LF_stdHit						( 0.35f ),
	LF_zHit							( 0.95f ),
	LF_zRandom						( 0.05f ),
	LF_maxRange						( 81.0f ),
	LF_decimation					( 5 ),
	LF_maxCorrsDistance				( 0.3f ),
	LF_useSquareDist				( false ),
	LF_alternateAverageMethod		( false ),

	MI_exponent						( 2.5f ),
	MI_skip_rays					( 10 ),
	MI_ratio_max_distance			( 1.5f ),

	rayTracing_useDistanceFilter	( true ),
	rayTracing_decimation			( 10 ),
	rayTracing_stdHit				( 1.0f ),

	consensus_takeEachRange			( 1 ),
	consensus_pow					( 5 ),
	OWA_weights						(100,1/100.0f),

	enableLikelihoodCache           ( true )
{
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const std::string &section)
{
	MRPT_LOAD_CONFIG_VAR_CAST(likelihoodMethod, int, TLikelihoodMethod, iniFile, section);

    enableLikelihoodCache               = iniFile.read_bool(section,"enableLikelihoodCache",enableLikelihoodCache);

	LF_stdHit							= iniFile.read_float(section,"LF_stdHit",LF_stdHit);
	LF_zHit								= iniFile.read_float(section,"LF_zHit",LF_zHit);
	LF_zRandom							= iniFile.read_float(section,"LF_zRandom",LF_zRandom);
	LF_maxRange							= iniFile.read_float(section,"LF_maxRange",LF_maxRange);
	LF_decimation						= iniFile.read_int(section,"LF_decimation",LF_decimation);
	LF_maxCorrsDistance					= iniFile.read_float(section,"LF_maxCorrsDistance",LF_maxCorrsDistance);
	LF_useSquareDist					= iniFile.read_bool(section,"LF_useSquareDist",LF_useSquareDist);
	LF_alternateAverageMethod			= iniFile.read_bool(section,"LF_alternateAverageMethod",LF_alternateAverageMethod);

	MI_exponent							= iniFile.read_float(section,"MI_exponent",MI_exponent);
	MI_skip_rays						= iniFile.read_int(section,"MI_skip_rays",MI_skip_rays);
	MI_ratio_max_distance				= iniFile.read_float(section,"MI_ratio_max_distance",MI_ratio_max_distance);

	rayTracing_useDistanceFilter		= iniFile.read_bool(section,"rayTracing_useDistanceFilter",rayTracing_useDistanceFilter);
	rayTracing_stdHit					= iniFile.read_float(section,"rayTracing_stdHit",rayTracing_stdHit);

	consensus_takeEachRange				= iniFile.read_int(section,"consensus_takeEachRange",consensus_takeEachRange);
	consensus_pow						= iniFile.read_float(section,"consensus_pow",consensus_pow);

	iniFile.read_vector(section,"OWA_weights",OWA_weights,OWA_weights);
}

/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::TLikelihoodOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
{
	out.printf("\n----------- [COccupancyGridMap2D::TLikelihoodOptions] ------------ \n\n");

	out.printf("likelihoodMethod                        = ");
	switch (likelihoodMethod)
	{
	case lmMeanInformation: out.printf("lmMeanInformation"); break;
	case lmRayTracing: out.printf("lmRayTracing"); break;
	case lmConsensus: out.printf("lmConsensus"); break;
	case lmCellsDifference: out.printf("lmCellsDifference"); break;
	case lmLikelihoodField_Thrun: out.printf("lmLikelihoodField_Thrun"); break;
	case lmLikelihoodField_II: out.printf("lmLikelihoodField_II"); break;
	case lmConsensusOWA: out.printf("lmConsensusOWA"); break;
	default:
		out.printf("UNKNOWN!!!"); break;
	}
	out.printf("\n");

	out.printf("enableLikelihoodCache                   = %c\n",	enableLikelihoodCache ? 'Y':'N');

	out.printf("LF_stdHit                               = %f\n",	LF_stdHit );
	out.printf("LF_zHit                                 = %f\n",	LF_zHit );
	out.printf("LF_zRandom                              = %f\n",	LF_zRandom );
	out.printf("LF_maxRange                             = %f\n",	LF_maxRange );
	out.printf("LF_decimation                           = %u\n",	LF_decimation );
	out.printf("LF_maxCorrsDistance                     = %f\n",	LF_maxCorrsDistance );
	out.printf("LF_useSquareDist                        = %c\n",	LF_useSquareDist ? 'Y':'N');
	out.printf("LF_alternateAverageMethod               = %c\n",	LF_alternateAverageMethod ? 'Y':'N');
	out.printf("MI_exponent                             = %f\n",	MI_exponent );
	out.printf("MI_skip_rays                            = %u\n",	MI_skip_rays );
	out.printf("MI_ratio_max_distance                   = %f\n",	MI_ratio_max_distance );
	out.printf("rayTracing_useDistanceFilter            = %c\n",	rayTracing_useDistanceFilter ? 'Y':'N');
	out.printf("rayTracing_decimation                   = %u\n",	rayTracing_decimation );
	out.printf("rayTracing_stdHit                       = %f\n",	rayTracing_stdHit );
	out.printf("consensus_takeEachRange                 = %u\n",	consensus_takeEachRange );
	out.printf("consensus_pow                           = %.02f\n", consensus_pow);
	out.printf("OWA_weights   = [");
	for (size_t i=0;i<OWA_weights.size();i++)
	{
		if (i<3 || i>(OWA_weights.size()-3))
			out.printf("%.03f ",OWA_weights[i]);
		else if (i==3 && OWA_weights.size()>6)
			out.printf(" ... ");
	}
	out.printf("] (size=%u)\n",(unsigned)OWA_weights.size());
	out.printf("\n");
}

/** Returns true if this map is able to compute a sensible likelihood function for this observation (i.e. an occupancy grid map cannot with an image).
 * \param obs The observation.
 * \sa computeObservationLikelihood
 */
bool COccupancyGridMap2D::internal_canComputeObservationLikelihood( const mrpt::obs::CObservation *obs ) const
{
	// Ignore laser scans if they are not planar or they are not
	//  at the altitude of this grid map:
	if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	{
		const CObservation2DRangeScan		*scan = static_cast<const CObservation2DRangeScan*>( obs );

		if (!scan->isPlanarScan(insertionOptions.horizontalTolerance))
			return false;
		if (insertionOptions.useMapAltitude &&
			fabs(insertionOptions.mapAltitude - scan->sensorPose.z() ) > 0.01 )
			return false;

		// OK, go on...
		return true;
	}
	else // Is not a laser scanner...
	{
		return false;
	}
}

