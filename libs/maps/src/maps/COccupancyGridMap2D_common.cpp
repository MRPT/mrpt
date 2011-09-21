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

// Force size_x being a multiple of 16 cells
//#define		ROWSIZE_MULTIPLE_16

#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CSimplePointsMap.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::poses;
using namespace std;


IMPLEMENTS_SERIALIZABLE(COccupancyGridMap2D, CMetricMap,mrpt::slam)

std::vector<float>		COccupancyGridMap2D::entropyTable;

static const float MAX_H = 0.69314718055994531f; // ln(2)

// Static lookup tables for log-odds
CLogOddsGridMapLUT<COccupancyGridMap2D::cellType>  COccupancyGridMap2D::m_logodd_lut;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
COccupancyGridMap2D::COccupancyGridMap2D(
	 float		min_x,
	 float		max_x,
	 float		min_y,
	 float		max_y,
	 float		resolution) :
		map(),
		size_x(0),size_y(0),
		x_min(),x_max(),y_min(),y_max(), resolution(),
		precomputedLikelihood(),
		precomputedLikelihoodToBeRecomputed(true),
		m_basis_map(),
		m_voronoi_diagram(),
		m_is_empty(true),
		voroni_free_threshold(),
		updateInfoChangeOnly(),
		insertionOptions(),
		likelihoodOptions(),
		likelihoodOutputs(),
		CriticalPointsList()
{
	MRPT_START
	setSize(min_x,max_x,min_y,max_y, resolution,0.5f );
	MRPT_END
}


/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
COccupancyGridMap2D::~COccupancyGridMap2D()
{
	freeMap();
}


/*---------------------------------------------------------------
						setSize
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::setSize(
	float		x_min,
	float		x_max,
	float		y_min,
	float		y_max,
	float		resolution,
	float		default_value)
{
	MRPT_START

	ASSERT_(resolution>0)
	ASSERT_(x_max>x_min && y_max>y_min)
	ASSERT_(default_value>=0 && default_value<=1)

    // Liberar primero:
    freeMap();

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	// Adjust sizes to adapt them to full sized cells acording to the resolution:
	x_min = resolution*round(x_min/resolution);
	y_min = resolution*round(y_min/resolution);
	x_max = resolution*round(x_max/resolution);
	y_max = resolution*round(y_max/resolution);

    // Set parameters:
    this->resolution = resolution;
    this->x_min = x_min;
    this->x_max = x_max;
    this->y_min = y_min;
    this->y_max = y_max;

	// Now the number of cells should be integers:
	size_x = round((x_max-x_min)/resolution);
	size_y = round((y_max-y_min)/resolution);

#ifdef	ROWSIZE_MULTIPLE_16
	// map rows must be 16 bytes aligned:
	if (0!=(size_x % 16))
	{
		size_x = ((size_x >> 4)+1) << 4;
        x_max = x_min + size_x * resolution;
	}
	size_x = round((x_max-x_min)/resolution);
	ASSERT_(0==(size_x % 16));
#endif

    // Cells memory:
    map.resize(size_x*size_y,p2l(default_value));

	// Free these buffers also:
	m_basis_map.clear();
	m_voronoi_diagram.clear();

	m_is_empty=true;

	MRPT_END
}

/*---------------------------------------------------------------
						ResizeGrid
  ---------------------------------------------------------------*/
void  COccupancyGridMap2D::resizeGrid(float new_x_min,float new_x_max,float new_y_min,float new_y_max,float new_cells_default_value, bool additionalMargin) MRPT_NO_THROWS
{
	unsigned int			extra_x_izq=0,extra_y_arr=0,new_size_x=0,new_size_y=0;
	std::vector<cellType>	new_map;

	if( new_x_min > new_x_max )
	{
		printf("[COccupancyGridMap2D::resizeGrid] Warning!! Ignoring call, since: x_min=%f  x_max=%f\n", new_x_min, new_x_max);
		return;
	}
	if( new_y_min > new_y_max )
	{
		printf("[COccupancyGridMap2D::resizeGrid] Warning!! Ignoring call, since: y_min=%f  y_max=%f\n", new_y_min, new_y_max);
		return;
	}

	// Required?
	if (new_x_min>=x_min &&
		new_y_min>=y_min &&
		new_x_max<=x_max &&
		new_y_max<=y_max)	return;

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	// Add an additional margin:
	if (additionalMargin)
	{
		if (new_x_min<x_min) new_x_min= floor(new_x_min-4);
		if (new_x_max>x_max) new_x_max= ceil(new_x_max+4);
		if (new_y_min<y_min) new_y_min= floor(new_y_min-4);
		if (new_y_max>y_max) new_y_max= ceil(new_y_max+4);
	}

	// We do not support grid shrinking... at least stay the same:
	new_x_min = min( new_x_min, x_min);
	new_x_max = max( new_x_max, x_max);
	new_y_min = min( new_y_min, y_min);
	new_y_max = max( new_y_max, y_max);


	// Adjust sizes to adapt them to full sized cells acording to the resolution:
	if (fabs(new_x_min/resolution - round(new_x_min/resolution))>0.05f )
		new_x_min = resolution*round(new_x_min/resolution);
	if (fabs(new_y_min/resolution - round(new_y_min/resolution))>0.05f )
		new_y_min = resolution*round(new_y_min/resolution);
	if (fabs(new_x_max/resolution - round(new_x_max/resolution))>0.05f )
		new_x_max = resolution*round(new_x_max/resolution);
	if (fabs(new_y_max/resolution - round(new_y_max/resolution))>0.05f )
		new_y_max = resolution*round(new_y_max/resolution);

	// Change size: 4 sides extensions:
	extra_x_izq = round((x_min-new_x_min) / resolution);
	extra_y_arr = round((y_min-new_y_min) / resolution);

	new_size_x = round((new_x_max-new_x_min) / resolution);
	new_size_y = round((new_y_max-new_y_min) / resolution);

	assert( new_size_x>=size_x+extra_x_izq );

#ifdef	ROWSIZE_MULTIPLE_16
	// map rows must be 16 bytes aligned:
	size_t old_new_size_x = new_size_x;  // Debug
	if (0!=(new_size_x % 16))
	{
		int size_x_incr = 16 - (new_size_x % 16);
        //new_x_max = new_x_min + new_size_x * resolution;
        new_x_max += size_x_incr * resolution;
	}
	new_size_x = round((new_x_max-new_x_min)/resolution);
	assert(0==(new_size_x % 16));
#endif

	// Reserve new mem block
	new_map.resize(new_size_x*new_size_y, p2l(new_cells_default_value));

	// Copy all the old map rows into the new map:
	{
		cellType  	*dest_ptr = &new_map[extra_x_izq + extra_y_arr*new_size_x];
		cellType  	*src_ptr  = &map[0];
		size_t 		row_size = size_x*sizeof(cellType);

		for (size_t y = 0;y<size_y;y++)
		{
#if defined(_DEBUG) || (MRPT_ALWAYS_CHECKS_DEBUG)
			assert( dest_ptr+row_size-1 <= &new_map[new_map.size()-1] );
			assert( src_ptr+row_size-1 <= &map[map.size()-1] );
#endif
			memcpy( dest_ptr, src_ptr, row_size );
			dest_ptr += new_size_x;
			src_ptr  += size_x;
		}
	}

	// Move new values into the new map:
	x_min = new_x_min;
	x_max = new_x_max;
	y_min = new_y_min;
	y_max = new_y_max;

	size_x = new_size_x;
	size_y = new_size_y;

	// Free old map, replace by new one:
	map.swap( new_map );

	// Free the other buffers:
	m_basis_map.clear();
	m_voronoi_diagram.clear();
}

/*---------------------------------------------------------------
						freeMap
  ---------------------------------------------------------------*/
void COccupancyGridMap2D::freeMap()
{
	MRPT_START

	// Free map and sectors
    map.clear();

	m_basis_map.clear();
	m_voronoi_diagram.clear();

    size_x=size_y=0;

	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;

	m_is_empty=true;

	MRPT_END
}



/*---------------------------------------------------------------
  Computes the entropy and related values of this grid map.
	out_H The target variable for absolute entropy, computed as:<br><center>H(map)=Sum<sub>x,y</sub>{ -p(x,y)·ln(p(x,y)) -(1-p(x,y))·ln(1-p(x,y)) }</center><br><br>
	out_I The target variable for absolute "information", defining I(x) = 1 - H(x)
	out_mean_H The target variable for mean entropy, defined as entropy per square meter: mean_H(map) = H(map) / (Map length x (meters))·(Map length y (meters))
	out_mean_I The target variable for mean information, defined as information per square meter: mean_I(map) = I(map) / (Map length x (meters))·(Map length y (meters))
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::computeEntropy( TEntropyInfo &info ) const
{
	unsigned long					i;
	float							h,p;
	std::vector<unsigned long>		histogram;


#ifdef	OCCUPANCY_GRIDMAP_CELL_SIZE_8BITS
	unsigned int								N = 256;
#else
	unsigned int								N = 65536;
#endif

	// Compute the entropy table: The entropy for each posible cell value
	// ----------------------------------------------------------------------
	if (entropyTable.size()!=N)
	{
		entropyTable.resize(N,0);
		for (i=0;i<N;i++)
		{
			p = l2p(static_cast<cellType>(i));
			h = H(p)+H(1-p);

			// Cell's probabilities rounding problem fixing:
			if (i==0 || i==(N-1)) h=0;
			if (h>(MAX_H - 1e-4)) h=MAX_H;

			entropyTable[i] = h;
		}
	}

	// Initialize the global results:
	info.H = info.I = 0;
	info.effectiveMappedCells = 0;


	info.H = info.I = 0;
	info.effectiveMappedCells = 0;
	for ( std::vector<cellType>::const_iterator it=map.begin();it!=map.end();++it)
	{
		cellTypeUnsigned  i = static_cast<cellTypeUnsigned>(*it);
		h = entropyTable[ i ];
		info.H+= h;
		if (h<(MAX_H-0.001f))
		{
			info.effectiveMappedCells++;
			info.I-=h;
		}
	}

	// The info: (See ref. paper EMMI in IROS 2006)
	info.I /= MAX_H;
	info.I += info.effectiveMappedCells;

	// Mean values:
	// ------------------------------------------
	info.effectiveMappedArea = info.effectiveMappedCells * resolution*resolution;
	if (info.effectiveMappedCells)
	{
		info.mean_H = info.H / info.effectiveMappedCells;
		info.mean_I = info.I / info.effectiveMappedCells;
	}
	else
	{
		info.mean_H = 0;
		info.mean_I = 0;
	}
}

/*---------------------------------------------------------------
					Entropy aux. function
 ---------------------------------------------------------------*/
double  COccupancyGridMap2D::H(double p)
{
	if (p==0 || p==1)	return 0;
	else				return -p*log(p);
}


/*---------------------------------------------------------------
							clear
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::internal_clear()
{
	setSize( -10,10,-10,10,getResolution());
	//resetFeaturesCache();
	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;
}

/*---------------------------------------------------------------
							fill
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::fill(float default_value)
{
	cellType		defValue = p2l( default_value );
	for (std::vector<cellType>::iterator	it=map.begin();it<map.end();++it)
		*it = defValue;
	// For the precomputed likelihood trick:
	precomputedLikelihoodToBeRecomputed = true;
	//resetFeaturesCache();
}

/*---------------------------------------------------------------
					updateCell
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::updateCell(int x,int y, float v)
{
	// Tip: if x<0, (unsigned)(x) will also be >>> size_x ;-)
	if (static_cast<unsigned int>(x)>=size_x || static_cast<unsigned int>(y)>=size_y)
		return;

	// Get the current contents of the cell:
	cellType	&theCell = map[x+y*size_x];

	// Compute the new Bayesian-fused value of the cell:
	if ( updateInfoChangeOnly.enabled )
	{
		float	old	= l2p(theCell);
		float		new_v	= 1 / ( 1 + (1-v)*(1-old)/(old*v) );
		updateInfoChangeOnly.cellsUpdated++;
		updateInfoChangeOnly.I_change+= 1-(H(new_v)+H(1-new_v))/MAX_H;
	}
	else
	{
		cellType obs = p2l(v);  // The observation: will be >0 for free, <0 for occupied.
		if (obs>0)
		{
			if ( theCell>(OCCGRID_CELLTYPE_MAX-obs) )
					theCell = OCCGRID_CELLTYPE_MAX; // Saturate
			else	theCell += obs;
		}
		else
		{
			if ( theCell<(OCCGRID_CELLTYPE_MIN-obs) )
					theCell = OCCGRID_CELLTYPE_MIN; // Saturate
			else	theCell += obs;
		}
	}
}


/*---------------------------------------------------------------
							subSample
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::subSample( int downRatio )
{
	std::vector<cellType>		newMap;

	ASSERT_(downRatio>0);

	resolution*=downRatio;

	int		newSizeX = round((x_max-x_min)/resolution);
	int		newSizeY = round((y_max-y_min)/resolution);

	newMap.resize(newSizeX*newSizeY);

	for (int x=0;x<newSizeX;x++)
	{
		for (int y=0;y<newSizeY;y++)
		{
			float	newCell = 0;

			for (int xx=0;xx<downRatio;xx++)
				for (int yy=0;yy<downRatio;yy++)
					newCell+= getCell(x*downRatio+xx, y*downRatio+yy);

			newCell /= (downRatio*downRatio);

			newMap[ x + y*newSizeX ] = p2l(newCell);
		}
	}


	setSize(x_min,x_max,y_min,y_max,resolution);
	map = newMap;


}



/*---------------------------------------------------------------
							computeMatchingWith
 ---------------------------------------------------------------*/
void  COccupancyGridMap2D::computeMatchingWith2D(
    const CMetricMap								*otherMap2,
    const CPose2D									&otherMapPose_,
    float									maxDistForCorrespondence,
    float									maxAngularDistForCorrespondence,
    const CPose2D									&angularDistPivotPoint,
    TMatchingPairList						&correspondences,
    float									&correspondencesRatio,
    float									*sumSqrDist,
    bool									onlyKeepTheClosest,
	bool									onlyUniqueRobust,
	const size_t                            decimation_other_map_points,
	const size_t          offset_other_map_points ) const
{
	MRPT_START

	ASSERT_ABOVE_(decimation_other_map_points,0)
	ASSERT_BELOW_(offset_other_map_points, decimation_other_map_points)
	const size_t incr = decimation_other_map_points; // Just to save some typing...

	ASSERT_(otherMap2->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap) ));
	const CPointsMap			*otherMap = static_cast<const CPointsMap*>(otherMap2);

	const TPose2D  otherMapPose = TPose2D(otherMapPose_);

	const size_t nLocalPoints = otherMap->getPointsCount();
	std::vector<float>				x_locals(nLocalPoints), y_locals(nLocalPoints),z_locals(nLocalPoints);


	const float sin_phi = sin(otherMapPose.phi);
	const float cos_phi = cos(otherMapPose.phi);

	size_t nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least
	size_t nTotalCorrespondences = 0;				// Total number of corrs
	float _sumSqrDist=0;

	// The number of cells to look around each point:
	const int cellsSearchRange = round( maxDistForCorrespondence / resolution );

	// Initially there are no correspondences:
	correspondences.clear();
	correspondencesRatio = 0;

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	// -----------------------------------------------------------
	float local_x_min =  std::numeric_limits<float>::max();
	float local_x_max = -std::numeric_limits<float>::max();
	float local_y_min =  std::numeric_limits<float>::max();
	float local_y_max = -std::numeric_limits<float>::max();

	const std::vector<float> & otherMap_pxs = otherMap->getPointsBufferRef_x();
	const std::vector<float> & otherMap_pys = otherMap->getPointsBufferRef_y();
	const std::vector<float> & otherMap_pzs = otherMap->getPointsBufferRef_z();

	// Translate all local map points:
	for (unsigned int localIdx=offset_other_map_points;localIdx<nLocalPoints;localIdx+=incr)
	{
		// Girar y desplazar cada uno de los puntos del local map:
		const float xx = x_locals[localIdx] = otherMapPose.x + cos_phi* otherMap_pxs[localIdx] - sin_phi* otherMap_pys[localIdx] ;
		const float yy = y_locals[localIdx] = otherMapPose.y + sin_phi* otherMap_pxs[localIdx] + cos_phi* otherMap_pys[localIdx] ;
		z_locals[localIdx] = /* otherMapPose.z +*/ otherMap_pzs[localIdx];

		// mantener el max/min de los puntos:
		local_x_min = min(local_x_min,xx);
		local_x_max = max(local_x_max,xx);
		local_y_min = min(local_y_min,yy);
		local_y_max = max(local_y_max,yy);
	}

	// If the local map is entirely out of the grid,
	//   do not even try to match them!!
	if (local_x_min> x_max ||
		local_x_max< x_min ||
		local_y_min> y_max ||
		local_y_max< y_min) return;		// Matching is NULL!


	const cellType thresholdCellValue = p2l(0.5f);

	// For each point in the other map:
	for (unsigned int localIdx=offset_other_map_points;localIdx<nLocalPoints;localIdx+=incr)
	{
		// Starting value:
		float maxDistForCorrespondenceSquared = square( maxDistForCorrespondence );

		// For speed-up:
		const float x_local = x_locals[localIdx];
		const float y_local = y_locals[localIdx];
		const float z_local = z_locals[localIdx];

		// Look for the occupied cell closest from the map point:
		float min_dist = 1e6;
		TMatchingPair		closestCorr;

		// Get the indexes of cell where the point falls:
		const int cx0=x2idx(x_local);
		const int cy0=y2idx(y_local);

		// Get the rectangle to look for into:
		const int cx_min = max(0, cx0 - cellsSearchRange );
		const int cx_max = min(static_cast<int>(size_x)-1, cx0 + cellsSearchRange );
		const int cy_min = max(0, cy0 - cellsSearchRange );
		const int cy_max = min(static_cast<int>(size_y)-1, cy0 + cellsSearchRange );

		// Will be set to true if a corrs. is found:
		bool thisLocalHasCorr = false;


		// Look in nearby cells:
		for (int cx=cx_min;cx<=cx_max;cx++)
		{
			for (int cy=cy_min;cy<=cy_max;cy++)
			{
				// Is an occupied cell?
				if ( map[cx+cy*size_x] < thresholdCellValue )//  getCell(cx,cy)<0.49)
				{
					const float residual_x = idx2x(cx)- x_local;
					const float residual_y = idx2y(cy)- y_local;

					// Compute max. allowed distance:
					maxDistForCorrespondenceSquared = square(
								maxAngularDistForCorrespondence * angularDistPivotPoint.distance2DTo(x_local,y_local) +
								maxDistForCorrespondence );

					// Square distance to the point:
					const float this_dist = square( residual_x ) + square( residual_y );

					if (this_dist<maxDistForCorrespondenceSquared)
					{
						if (!onlyKeepTheClosest)
						{
							// save the correspondence:
							nTotalCorrespondences++;
							TMatchingPair   mp;
							mp.this_idx = cx+cy*size_x;
							mp.this_x = idx2x(cx);
							mp.this_y = idx2y(cy);
							mp.this_z = z_local;
							mp.other_idx = localIdx;
							mp.other_x = otherMap_pxs[localIdx];
							mp.other_y = otherMap_pys[localIdx];
							mp.other_z = otherMap_pzs[localIdx];
							correspondences.push_back( mp );
						}
						else
						{
							// save the closest only:
							if (this_dist<min_dist)
							{
								min_dist = this_dist;

								closestCorr.this_idx = cx+cy*size_x;
								closestCorr.this_x = idx2x(cx);
								closestCorr.this_y = idx2y(cy);
								closestCorr.this_z = z_local;
								closestCorr.other_idx = localIdx;
								closestCorr.other_x = otherMap_pxs[localIdx];
								closestCorr.other_y = otherMap_pys[localIdx];
								closestCorr.other_z = otherMap_pzs[localIdx];
							}
						}

						// At least one:
						thisLocalHasCorr = true;
					}
				}
			}
		} // End of find closest nearby cell

		// save the closest correspondence:
		if (onlyKeepTheClosest && (min_dist<maxDistForCorrespondenceSquared))
		{
			nTotalCorrespondences++;
			correspondences.push_back( closestCorr );
		}

		// At least one corr:
		if (thisLocalHasCorr)
		{
			nOtherMapPointsWithCorrespondence++;

			// Accumulate the MSE:
			_sumSqrDist+= min_dist;
		}

	}	// End "for each local point"...

	correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints/incr);

	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	if (sumSqrDist) *sumSqrDist = _sumSqrDist;


//		os::fclose(fDebug);

	MRPT_END

}


/*---------------------------------------------------------------
					isEmpty
  ---------------------------------------------------------------*/
bool  COccupancyGridMap2D::isEmpty() const
{
	return m_is_empty;
}



/*---------------------------------------------------------------
				operator <
  ---------------------------------------------------------------*/
bool mrpt::slam::operator < (const COccupancyGridMap2D::TPairLikelihoodIndex &e1, const COccupancyGridMap2D::TPairLikelihoodIndex &e2)
{
	return e1.first > e2.first;
}



/*---------------------------------------------------------------
				computePathCost
  ---------------------------------------------------------------*/
float  COccupancyGridMap2D::computePathCost( float x1, float y1, float x2, float y2 ) const
{
	float	sumCost = 0;

	float	dist = sqrt( square(x1-x2)+square(y1-y2) );
	int	nSteps = round(1.5f * dist / resolution);

	for (int i=0;i<nSteps;i++)
	{
		float	x = x1 + (x2-x1)*i/static_cast<float>(nSteps);
		float	y = y1 + (y2-y1)*i/static_cast<float>(nSteps);
		sumCost += getPos( x,y );
	}

	if (nSteps)
			return sumCost/static_cast<float>(nSteps);
	else	return 0;
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
float  COccupancyGridMap2D::compute3DMatchingRatio(
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
