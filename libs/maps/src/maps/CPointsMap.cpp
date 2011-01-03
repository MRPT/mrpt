/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
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

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/math/geometry.h>

#include <mrpt/slam/CPointsMap.h>

#include <mrpt/opengl/CPointCloud.h>

using namespace mrpt::poses;
using namespace mrpt::slam;
using namespace std;


float mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE = 3.0f;


IMPLEMENTS_VIRTUAL_SERIALIZABLE(CPointsMap, CMetricMap,mrpt::slam)


float CPointsMap::COLOR_3DSCENE_R = 0;
float CPointsMap::COLOR_3DSCENE_G = 0;
float CPointsMap::COLOR_3DSCENE_B = 1;

extern CStartUpClassesRegister  mrpt_maps_class_reg;
const int dumm = mrpt_maps_class_reg.do_nothing(); // Avoid compiler removing this class in static linking

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPointsMap::CPointsMap() :
	x(),y(),z(),pointWeight(),
	m_largestDistanceFromOrigin(0),
	insertionOptions(),
	likelihoodOptions()
{
	mark_as_modified();
}

/*---------------------------------------------------------------
						Destructor
  ---------------------------------------------------------------*/
CPointsMap::~CPointsMap()
{

}

/*---------------------------------------------------------------
					save2D_to_text_file
  Save to a text file. In each line there are a point coordinates.
    Returns false if any error occured, true elsewere.
  ---------------------------------------------------------------*/
bool  CPointsMap::save2D_to_text_file(const string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return false;

	for (unsigned int i=0;i<x.size();i++)
		os::fprintf(f,"%f %f\n",x[i],y[i]);

	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
					save3D_to_text_file
  Save to a text file. In each line there are a point coordinates.
    Returns false if any error occured, true elsewere.
  ---------------------------------------------------------------*/
bool  CPointsMap::save3D_to_text_file(const string &file) const
{
	FILE	*f=os::fopen(file.c_str(),"wt");
	if (!f) return false;

	for (unsigned int i=0;i<x.size();i++)
		os::fprintf(f,"%f %f %f\n",x[i],y[i],z[i]);

	os::fclose(f);
	return true;
}

/*---------------------------------------------------------------
					getPointsCount
  ---------------------------------------------------------------*/
size_t  CPointsMap::getPointsCount() const
{
	return x.size();
}

/*---------------------------------------------------------------
						size
  ---------------------------------------------------------------*/
size_t CPointsMap::size() const
{
	return x.size();
}


/*---------------------------------------------------------------
					getPoint
			Access to stored points coordinates:
  ---------------------------------------------------------------*/
unsigned long  CPointsMap::getPoint(size_t index,CPoint2D &p) const
{
	if (index>=x.size())
		THROW_EXCEPTION("Index out of bounds");

	p.x( x[index] );
	p.y( y[index] );

	return pointWeight[index];
}

unsigned long  CPointsMap::getPoint(size_t index,CPoint3D &p) const
{
	if (index>=x.size())
		THROW_EXCEPTION("Index out of bounds");

	p.x( x[index] );
	p.y( y[index] );
	p.z( z[index] );

	return pointWeight[index];
}

unsigned long  CPointsMap::getPoint(size_t index,TPoint3D &p) const
{
	if (index>=x.size())
		THROW_EXCEPTION("Index out of bounds");

	p.x= x[index];
	p.y= y[index];
	p.z= z[index];

	return pointWeight[index];
}

unsigned long  CPointsMap::getPoint(size_t index,TPoint2D &p) const
{
	if (index>=x.size())
		THROW_EXCEPTION("Index out of bounds");

	p.x= x[index];
	p.y= y[index];

	return pointWeight[index];
}

unsigned long  CPointsMap::getPoint(size_t index,float &x,float &y) const
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	x = this->x[index];
	y = this->y[index];

	return pointWeight[index];
}
unsigned long  CPointsMap::getPoint(size_t index,float &x,float &y,float &z) const
{
	if (index>=this->x.size())
		THROW_EXCEPTION("Index out of bounds");

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	return pointWeight[index];
}

/*---------------------------------------------------------------
						getPointsBuffer
 ---------------------------------------------------------------*/
void  CPointsMap::getPointsBuffer( size_t &outPointsCount, const float *&xs, const float *&ys, const float *&zs ) const
{
	outPointsCount = getPointsCount();

	if (outPointsCount>0)
	{
		xs = &x[0];
		ys = &y[0];
		zs = &z[0];
	}
	else
	{
		xs = ys = zs = NULL;
	}
}

/*---------------------------------------------------------------
						clipOutOfRangeInZ
 ---------------------------------------------------------------*/
void  CPointsMap::clipOutOfRangeInZ(float zMin, float zMax)
{
	size_t			i,n=getPointsCount();
	vector<bool>	deletionMask;

	// The deletion mask:
	deletionMask.resize(n);

	// Compute it:
	for (i=0;i<n;i++)
		deletionMask[i] = ( z[i]<zMin || z[i]>zMax );

	// Perform deletion:
	applyDeletionMask(deletionMask);

	mark_as_modified();
}


/*---------------------------------------------------------------
						clipOutOfRange
 ---------------------------------------------------------------*/
void  CPointsMap::clipOutOfRange(const CPoint2D	&point, float maxRange)
{
	size_t			i,n=getPointsCount();
	vector<bool>	deletionMask;

	// The deletion mask:
	deletionMask.resize(n);

	// Compute it:
	for (i=0;i<n;i++)
		deletionMask[i] = point.distance2DTo( x[i],y[i] ) > maxRange;

	// Perform deletion:
	applyDeletionMask(deletionMask);

	mark_as_modified();
}

/*---------------------------------------------------------------
					ComputeMatchingWith

 Computes the matching between two 2D points maps.
   This includes finding:
		- The set of points pairs in each map
		- The mean squared distance between corresponding pairs.
   This method is the most time critical one in the ICP algorithm.

 otherMap					  [IN] The other map to compute the matching with.
 otherMapPose				  [IN] The pose of the other map as seen from "this".
 maxDistForCorrespondenceSqrt [IN] Maximum 2D distance between two points to be matched.
 maxAngErrorForCorrespondence [IN] Maximum angular distance in radians to allow far points to be matched.
 correspondences			  [OUT] The detected matchings pairs.Value at [i] is the index in this map
									 corresponding to point "i" in the "other" map, or -1 if none.
 nCorrespondences			  [OUT] The number of correct correspondences.
 auxInfo					  [IN/OUT] Auxiliar structure.Initialize "loop_count" to 0.
 sumSqrDist					  [OUT] The sum of all matched points squared distances.If undesired,
									 set to NULL, as default.
 covariance					  [OUT] The resulting matching covariance 3x3 matrix, or NULL if undesired.
  ---------------------------------------------------------------*/
void  CPointsMap::computeMatchingWith2D(
    const CMetricMap								*otherMap2,
    const CPose2D									&otherMapPose_,
    float									maxDistForCorrespondence,
    float									maxAngularDistForCorrespondence,
    const CPose2D									&angularDistPivotPoint,
    TMatchingPairList						&correspondences,
    float									&correspondencesRatio,
    float									*sumSqrDist	,
    bool									onlyKeepTheClosest,
	bool									onlyUniqueRobust) const
{
	MRPT_START;

	ASSERT_(otherMap2->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap) ));
	const CPointsMap		*otherMap = static_cast<const CPointsMap*>( otherMap2 );

	const TPose2D	otherMapPose(otherMapPose_.x(),otherMapPose_.y(),otherMapPose_.phi());

	size_t					nLocalPoints = otherMap->size();
	size_t					nGlobalPoints = this->size();
	float					_sumSqrDist=0;
	size_t					_sumSqrCount = 0;
	size_t					nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

	float					local_x_min= std::numeric_limits<float>::max(), local_x_max= -std::numeric_limits<float>::max();
	float					global_x_min=std::numeric_limits<float>::max(), global_x_max= -std::numeric_limits<float>::max();
	float					local_y_min= std::numeric_limits<float>::max(), local_y_max= -std::numeric_limits<float>::max();
	float					global_y_min=std::numeric_limits<float>::max(), global_y_max= -std::numeric_limits<float>::max();

	double					maxDistForCorrespondenceSquared;
	float			        x_local, y_local,z_local;
	unsigned int			globalIdx,localIdx;

	vector<float>					x_locals(nLocalPoints),y_locals(nLocalPoints),z_locals(nLocalPoints);

	float 		*x_locals_it,*y_locals_it,*z_locals_it;
	const float *x_other_it,*y_other_it,*z_other_it,*x_global_it,*y_global_it; //,*z_global_it;

	// No correspondences initially:
	correspondences.clear();
	correspondencesRatio = 0;

	// Hay mapa global?
	if (!nGlobalPoints) return;  // No

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	double	sin_phi = sin(otherMapPose.phi);
	double	cos_phi = cos(otherMapPose.phi);

	// Do matching only there is any chance of the two maps to overlap:
	// -----------------------------------------------------------

	// Translate and rotate all local points:
	for (   localIdx=0,
		    x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&otherMap->x[0],
			y_other_it=&otherMap->y[0],
			z_other_it=&otherMap->z[0];
			localIdx<nLocalPoints;
			localIdx++)
	{
		// Translate and rotate each point in the "other" map:
		float  x_other = *x_other_it++;
		float  y_other = *y_other_it++;

		float  x_local = *x_locals_it++  = otherMapPose.x + cos_phi * x_other - sin_phi * y_other ;
		float  y_local = *y_locals_it++  = otherMapPose.y + sin_phi * x_other + cos_phi * y_other;
		*z_locals_it++ = *z_other_it++;

		// Find the bounding box:
		local_x_min = min(local_x_min,x_local);
		local_x_max = max(local_x_max,x_local);
		local_y_min = min(local_y_min,y_local);
		local_y_max = max(local_y_max,y_local);
	}

	// Find the bounding box:
	for (   globalIdx=0,
		    x_global_it=&x[0],
		    y_global_it=&y[0];
			globalIdx<nGlobalPoints;
			globalIdx++)
	{
		float global_x = *x_global_it++;
		float global_y = *y_global_it++;

		global_x_min = min(global_x_min,global_x);
		global_x_max = max(global_x_max,global_x);
		global_y_min = min(global_y_min,global_y);
		global_y_max = max(global_y_max,global_y);
	}

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	if (local_x_min>global_x_max ||
		local_x_max<global_x_min ||
		local_y_min>global_y_max ||
		local_y_max<global_y_min) return;	// No hace falta hacer matching,
											//   porque es de CERO.

	// Loop for each point in local map:
	// --------------------------------------------------
	for ( localIdx=0,
			x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&otherMap->x[0],
			y_other_it=&otherMap->y[0],
			z_other_it=&otherMap->z[0];
			localIdx<nLocalPoints;
			x_locals_it++,y_locals_it++,z_locals_it++,x_other_it++,y_other_it++,z_other_it++,localIdx++ )
	{
		// For speed-up:
		x_local = *x_locals_it;
		y_local = *y_locals_it;
		z_local = *z_locals_it;

		// Find all the matchings in the requested distance:
		TMatchingPair		p,closestPair;

		// KD-TREE implementation
#if 1
		// Use a KD-tree to look for the nearnest neighbor of:
		//   (x_local, y_local, z_local)
		// In "this" (global/reference) points map.

		p.this_idx = kdTreeClosestPoint2D(
			x_local,  y_local,  // Look closest to this guy
			p.this_x, p.this_y, // save here the closest match
			p.errorSquareAfterTransformation // save here the min. distance squared
			);

		// Compute max. allowed distance:
		maxDistForCorrespondenceSquared = square(
					maxAngularDistForCorrespondence * angularDistPivotPoint.distance2DTo(x_local,y_local) +
					maxDistForCorrespondence );

		// Distance below the threshold??
		if ( p.errorSquareAfterTransformation < maxDistForCorrespondenceSquared )
		{
			// Save all the correspondences??
			p.this_z = z[p.this_idx];
			p.other_idx = localIdx;
			p.other_x = *x_other_it;
			p.other_y = *y_other_it;
			p.other_z = *z_other_it;

			// save the correspondence:
			correspondences.push_back( p );

			// At least one:
			nOtherMapPointsWithCorrespondence++;

			// Accumulate the MSE:
			_sumSqrDist+= p.errorSquareAfterTransformation;
			_sumSqrCount++;
		}


#else // old implementation

		bool thisLocalHasCorr = false;

		float min_dist = 1e6;
		// Loop for global points:
		// ----------------------------------
		const float *z_global_it;

		for (   globalIdx=0,
				x_global_it=&x[0],
				y_global_it=&y[0],
				z_global_it=&z[0];
				globalIdx<nGlobalPoints;
//					x_global_it=x.begin(),y_global_it=y.begin(),z_global_it=z.begin();
//					x_global_it!=x.end();
			x_global_it++,y_global_it++,z_global_it++,globalIdx++)
		{
			float residual_x = *x_global_it - x_local;
			float residual_y = *y_global_it - y_local;

			// Compute max. allowed distance:
			maxDistForCorrespondenceSquared = square(
						maxAngularDistForCorrespondence * angularDistPivotPoint.distance2DTo(x_local,y_local) +
						maxDistForCorrespondence );

			float this_dist = square(*x_global_it - x_local) +
						square(*y_global_it - y_local);
						//square(*z_global_it - z_local);

			if (this_dist<maxDistForCorrespondenceSquared)
			{
				// Save all the correspondences??
				p.this_idx = globalIdx;
				p.this_x = *x_global_it;
				p.this_y = *y_global_it;
				p.this_z = *z_global_it;
				p.other_idx = localIdx;
				p.other_x = *x_other_it;
				p.other_y = *y_other_it;
				p.other_z = *z_other_it;

				p.errorSquareAfterTransformation = this_dist;

				if (!onlyKeepTheClosest)
				{
					// save the correspondence:
					correspondences.push_back( p );
				}
				else
				{
					// Or try to find the closest only?
					// Keep the min. distance:
					if (this_dist<min_dist)
						closestPair = p;
				}

				if (this_dist<min_dist)
						min_dist = this_dist;

				// At least one:
				thisLocalHasCorr = true;
			}

		} // End for each global point:

		// At least one corr:
		if (thisLocalHasCorr)
		{
			nOtherMapPointsWithCorrespondence++;

			// Accumulate the MSE:
			_sumSqrDist+= min_dist;
			_sumSqrCount++;

			// Save the closest only?
			if (onlyKeepTheClosest)
			{
				correspondences.push_back( closestPair );
			}
		}
#endif

	} // For each local point

	// Additional consistency filter: "onlyKeepTheClosest" up to now
	//  led to just one correspondence for each "local map" point, but
	//  many of them may have as corresponding pair the same "global point"!!
	// -------------------------------------------------------------------------
	if (onlyUniqueRobust)
	{
		if (!onlyKeepTheClosest)  THROW_EXCEPTION("ERROR: onlyKeepTheClosest must be also set to true when onlyUniqueRobust=true.")

		vector<TMatchingPairPtr>	bestMatchForThisMap( nGlobalPoints, TMatchingPairPtr(NULL) );
		TMatchingPairList::iterator it;

		//   1) Go through all the correspondences and keep the best corresp.
		//       for each "global map" (this) point.
		for (it=correspondences.begin();it!=correspondences.end();it++)
		{
			if (!bestMatchForThisMap[it->this_idx])
			{
				bestMatchForThisMap[it->this_idx] = &(*it);
			}
			else
			{
				if ( it->errorSquareAfterTransformation < bestMatchForThisMap[it->this_idx]->errorSquareAfterTransformation )
					bestMatchForThisMap[it->this_idx] = &(*it);
			}
		}

		//   2) Go again through the list of correspondences and remove those
		//       who are not the best one for their corresponding global map.
		for (it=correspondences.begin();it!=correspondences.end();)
		{
			if ( bestMatchForThisMap[it->this_idx] != &(*it) )
			{
				// Remove & get next:
				it = correspondences.erase( it );
			}
			else // Do not remove, get next:
			{
				it++;
			}
		}
	} // end of additional consistency filer for "onlyKeepTheClosest"

	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	if (sumSqrDist)
	{
		if (_sumSqrCount)
				*sumSqrDist = _sumSqrDist / static_cast<double>(_sumSqrCount);
		else	*sumSqrDist = 0;
	}

	// The ratio of points in the other map with corrs:
	correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

	MRPT_END;
}

/*---------------------------------------------------------------
				changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointsMap::changeCoordinatesReference(const CPose2D	&newBase)
{
	CPoint2D		l,g;

	size_t i, N = x.size();

	for (i=0;i<N;i++)
	{
		getPoint(i,l);
		g = newBase + l;
		setPoint(i,g);
	}

	mark_as_modified();
}

/*---------------------------------------------------------------
				changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointsMap::changeCoordinatesReference(const CPose3D	&newBase)
{
	CPoint3D		l,g;

	size_t i, N = x.size();

	for (i=0;i<N;i++)
	{
		getPoint(i,l);
		g = newBase + l;
		setPoint(i,g);
	}

	mark_as_modified();
}

/*---------------------------------------------------------------
				changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointsMap::changeCoordinatesReference(const CPointsMap &other, const CPose3D &newBase)
{
	copyFrom(other);
	changeCoordinatesReference( newBase );
}


/*---------------------------------------------------------------
				isEmpty
 ---------------------------------------------------------------*/
bool  CPointsMap::isEmpty() const
{
	return x.empty();
}

/*---------------------------------------------------------------
				TInsertionOptions
 ---------------------------------------------------------------*/
CPointsMap::TInsertionOptions::TInsertionOptions() :
	minDistBetweenLaserPoints	( 0.02f),
	addToExistingPointsMap		( true),
	also_interpolate			( false),
	disableDeletion				( true),
	fuseWithExisting			( false),
	isPlanarMap					( false),
	horizontalTolerance			( DEG2RAD(0.05) ),
	maxDistForInterpolatePoints	( 2.0f )
{
}

CPointsMap::TLikelihoodOptions::TLikelihoodOptions() :
	sigma_dist			( 0.05 ),
	max_corr_distance	( 1.0 ),
	decimation			( 10 )
{

}

void CPointsMap::TLikelihoodOptions::writeToStream(CStream &out) const
{
	const int8_t version = 0;
	out << version;
	out << sigma_dist << max_corr_distance << decimation;
}

void CPointsMap::TLikelihoodOptions::readFromStream(CStream &in)
{
	int8_t version;
	in >> version;
	switch(version)
	{
		case 0:
		{
			in >> sigma_dist >> max_corr_distance >> decimation;
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	}
}


/*---------------------------------------------------------------
					dumpToTextStream
  ---------------------------------------------------------------*/
void  CPointsMap::TInsertionOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CPointsMap::TInsertionOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(minDistBetweenLaserPoints,double);
	LOADABLEOPTS_DUMP_VAR(maxDistForInterpolatePoints,double);
	LOADABLEOPTS_DUMP_VAR_DEG(horizontalTolerance);

	LOADABLEOPTS_DUMP_VAR(addToExistingPointsMap,bool);
	LOADABLEOPTS_DUMP_VAR(also_interpolate,bool);
	LOADABLEOPTS_DUMP_VAR(disableDeletion,bool);
	LOADABLEOPTS_DUMP_VAR(fuseWithExisting,bool);
	LOADABLEOPTS_DUMP_VAR(isPlanarMap,bool);

	out.printf("\n");
}

void  CPointsMap::TLikelihoodOptions::dumpToTextStream(CStream	&out) const
{
	out.printf("\n----------- [CPointsMap::TLikelihoodOptions] ------------ \n\n");

	LOADABLEOPTS_DUMP_VAR(sigma_dist,double);
	LOADABLEOPTS_DUMP_VAR(max_corr_distance,double);
	LOADABLEOPTS_DUMP_VAR(decimation,int);
}

/*---------------------------------------------------------------
					loadFromConfigFile
  ---------------------------------------------------------------*/
void  CPointsMap::TInsertionOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const string &section)
{
	MRPT_LOAD_CONFIG_VAR(minDistBetweenLaserPoints,float, iniFile,section);
	MRPT_LOAD_CONFIG_VAR_DEGREES(horizontalTolerance,  iniFile, section);

	MRPT_LOAD_CONFIG_VAR(addToExistingPointsMap,			bool,  iniFile,section);
	MRPT_LOAD_CONFIG_VAR(also_interpolate,			bool,  iniFile,section);
	MRPT_LOAD_CONFIG_VAR(disableDeletion,			bool,  iniFile,section);
	MRPT_LOAD_CONFIG_VAR(fuseWithExisting,			bool,  iniFile,section);
	MRPT_LOAD_CONFIG_VAR(isPlanarMap,			bool,  iniFile,section);

	MRPT_LOAD_CONFIG_VAR(maxDistForInterpolatePoints,	float, iniFile,section);
}

void  CPointsMap::TLikelihoodOptions::loadFromConfigFile(
	const mrpt::utils::CConfigFileBase  &iniFile,
	const string &section)
{
	MRPT_LOAD_CONFIG_VAR(sigma_dist,double,iniFile,section);
	MRPT_LOAD_CONFIG_VAR(max_corr_distance,double,iniFile,section);
	MRPT_LOAD_CONFIG_VAR(decimation,int,iniFile,section);
}

/*---------------------------------------------------------------
						getAs3DObject
---------------------------------------------------------------*/
void  CPointsMap::getAs3DObject( mrpt::opengl::CSetOfObjectsPtr	&outObj ) const
{
	if (m_disableSaveAs3DObject)
		return;

	opengl::CPointCloudPtr  obj = opengl::CPointCloud::Create();

	obj->loadFromPointsMap(this);
	obj->setColor( CPointsMap::COLOR_3DSCENE_R, CPointsMap::COLOR_3DSCENE_G, CPointsMap::COLOR_3DSCENE_B, 1 );
	obj->setPointSize( mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE );
	obj->enableColorFromZ(true);

	obj->setGradientColors( TColorf(0.0,0,0), TColorf(CPointsMap::COLOR_3DSCENE_R,CPointsMap::COLOR_3DSCENE_G,CPointsMap::COLOR_3DSCENE_B));

	outObj->insert(obj);
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
---------------------------------------------------------------*/
float  CPointsMap::compute3DMatchingRatio(
		const CMetricMap								*otherMap2,
		const CPose3D							&otherMapPose,
		float									minDistForCorr,
		float									minMahaDistForCorr
		) const
{
	MRPT_UNUSED_PARAM(minMahaDistForCorr);
	MRPT_START;

	// Do have the other map any points map??
	const CPointsMap *otherMap = otherMap2->getAsSimplePointsMap();

	// There is not a points map to compare to!
	if (!otherMap) return 0;


	size_t					nLocalPoints = otherMap->size();
	size_t					nGlobalPoints = this->size();
	size_t					nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least
	vector<float>			x_locals,y_locals,z_locals;
	vector<float>::iterator	x_locals_it,y_locals_it,z_locals_it;
	vector<float>::const_iterator	x_other_it,y_other_it,z_other_it,x_global_it,y_global_it,z_global_it;

	float					local_x_min= std::numeric_limits<float>::max(), local_x_max= -std::numeric_limits<float>::max();
	float					global_x_min=std::numeric_limits<float>::max(), global_x_max= -std::numeric_limits<float>::max();
	float					local_y_min= std::numeric_limits<float>::max(), local_y_max= -std::numeric_limits<float>::max();
	float					global_y_min=std::numeric_limits<float>::max(), global_y_max= -std::numeric_limits<float>::max();
	float					local_z_min= std::numeric_limits<float>::max(), local_z_max= -std::numeric_limits<float>::max();
	float					global_z_min=std::numeric_limits<float>::max(), global_z_max= -std::numeric_limits<float>::max();

	float					maxDistForCorrespondenceSquared = square(minDistForCorr);
	float					x_local, y_local,z_local;
	size_t					localIdx;

	// There are points here?
	if (!nGlobalPoints) return 0;  // No

	// There are points there?
	if (!nLocalPoints)  return 0;  // No

	// The transformation:

	CMatrixDouble44 pose3DMatrix;
	otherMapPose.getHomogeneousMatrix(pose3DMatrix);

	float		Tx  = pose3DMatrix.get_unsafe(0,3);
	float		Ty  = pose3DMatrix.get_unsafe(1,3);
	float		Tz  = pose3DMatrix.get_unsafe(2,3);

	// ---------------------------------------------------------------------------------------------------------------
	// Is there any "contact" between the spheres that contain all the points from each map after translating them??
	// (Note that we can avoid computing the rotation of all the points if this test fail, with a great speed up!)
	// ---------------------------------------------------------------------------------------------------------------
	if ( sqrt( square(Tx) + square(Ty) + square(Tz) ) >= ( getLargestDistanceFromOrigin()+otherMap->getLargestDistanceFromOrigin() ) )
		return 0;		// There is no contact!

	float		R11 = pose3DMatrix.get_unsafe(0,0);
	float		R12 = pose3DMatrix.get_unsafe(0,1);
	float		R13 = pose3DMatrix.get_unsafe(0,2);

	float		R21 = pose3DMatrix.get_unsafe(1,0);
	float		R22 = pose3DMatrix.get_unsafe(1,1);
	float		R23 = pose3DMatrix.get_unsafe(1,2);

	float		R31 = pose3DMatrix.get_unsafe(2,0);
	float		R32 = pose3DMatrix.get_unsafe(2,1);
	float		R33 = pose3DMatrix.get_unsafe(2,2);

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	// Transladar y rotar ya todos los puntos locales
	// -----------------------------------------------------------
	x_locals.resize(nLocalPoints);
	y_locals.resize(nLocalPoints);
	z_locals.resize(nLocalPoints);

	for ( x_locals_it=x_locals.begin(),
			y_locals_it=y_locals.begin(),
			z_locals_it=z_locals.begin(),
			x_other_it=otherMap->x.begin(),
			y_other_it=otherMap->y.begin(),
			z_other_it=otherMap->z.begin();
			x_other_it!=otherMap->x.end();
			x_locals_it++,y_locals_it++,z_locals_it++,x_other_it++,y_other_it++,z_other_it++)
	{
		// Girar y desplazar cada uno de los puntos del local map:
		*x_locals_it = Tx + *x_other_it*R11 + *y_other_it*R12+ *z_other_it*R13;
		*y_locals_it = Ty + *x_other_it*R21 + *y_other_it*R22+ *z_other_it*R23;
		*z_locals_it = Tz + *x_other_it*R31 + *y_other_it*R32+ *z_other_it*R33;

		// Find the bounding box:
		local_x_min = min(local_x_min,*x_locals_it);
		local_x_max = max(local_x_max,*x_locals_it);
		local_y_min = min(local_y_min,*y_locals_it);
		local_y_max = max(local_y_max,*y_locals_it);
		local_z_min = min(local_z_min,*z_locals_it);
		local_z_max = max(local_z_max,*z_locals_it);
	}

	// Find the bounding box:
	for ( x_global_it=x.begin(),y_global_it=y.begin(),z_global_it=z.begin();
			x_global_it!=x.end();
			x_global_it++,y_global_it++,z_global_it++)
	{
		global_x_min = min(global_x_min,*x_global_it);
		global_x_max = max(global_x_max,*x_global_it);
		global_y_min = min(global_y_min,*y_global_it);
		global_y_max = max(global_y_max,*y_global_it);
		global_z_min = min(global_z_min,*z_global_it);
		global_z_max = max(global_z_max,*z_global_it);
	}

	// Do the maps overlap??
	if (local_x_min>global_x_max || local_x_max<global_x_min ||
		local_y_min>global_y_max || local_y_max<global_y_min ||
		local_z_min>global_z_max || local_z_max<global_z_min ) return 0;

	// Loop for each point in local map:
	// --------------------------------------------------
	for ( localIdx=0,
			x_locals_it=x_locals.begin(),
			y_locals_it=y_locals.begin(),
			z_locals_it=z_locals.begin(),
			x_other_it=otherMap->x.begin(),
			y_other_it=otherMap->y.begin(),
			z_other_it=otherMap->z.begin();
			x_locals_it!=x_locals.end();
			x_locals_it++,y_locals_it++,z_locals_it++,x_other_it++,y_other_it++,z_other_it++,localIdx++ )
	{
		// For speed-up:
		x_local = *x_locals_it;
		y_local = *y_locals_it;
		z_local = *z_locals_it;

		bool thisLocalHasCorr = false;

#if 1
		// Use a KD-tree to look for the nearnest neighbor of:
		//   (x_local, y_local, z_local)
		// In "this" (global/reference) points map.

		float errorSquareAfterTransformation, this_x, this_y, this_z;

		//size_t this_idx =
		kdTreeClosestPoint3D(
			x_local,  y_local, z_local, // Look closest to this guy
			this_x, this_y, this_z, // save here the closest match
			errorSquareAfterTransformation // save here the min. distance squared
			);

		// Distance below the threshold??
		thisLocalHasCorr  = errorSquareAfterTransformation < maxDistForCorrespondenceSquared;

#else
		size_t					globalIdx;
		float					residual_x,residual_y,residual_z;
		float					min_dist;

		// Find all the matchings in the requested distance:
		min_dist = 1e6;
		// Loop for global points:
		// ----------------------------------
		for ( globalIdx=0,
				x_global_it=x.begin(),y_global_it=y.begin(),z_global_it=z.begin();
				x_global_it!=x.end() && !thisLocalHasCorr;
			x_global_it++,y_global_it++,z_global_it++,globalIdx++)
		{
			residual_x = *x_global_it - x_local;
			residual_y = *y_global_it - y_local;
			residual_z = *z_global_it - z_local;

			if ( square(residual_x)+square(residual_y)+square(residual_z) < maxDistForCorrespondenceSquared )
				thisLocalHasCorr = true;

		} // End for-global points
#endif

		// At least one corr?
		if (thisLocalHasCorr) nOtherMapPointsWithCorrespondence++;

	} // For each local point:

	// The ratio of points in the other map with corrs:
	return nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

	MRPT_END;
}

/*---------------------------------------------------------------
						getLargestDistanceFromOrigin
---------------------------------------------------------------*/
float  CPointsMap::getLargestDistanceFromOrigin() const
{
	// Updated?
	if (!m_largestDistanceFromOriginIsUpdated)
	{
		// NO: Update it:
		vector<float>::const_iterator    X,Y,Z;
		float	maxDistSq = 0, d;
		for (X=x.begin(),Y=y.begin(),Z=z.begin();X!=x.end();X++,Y++,Z++)
		{
			d = square(*X)+square(*Y)+square(*Z);
			maxDistSq = max( d, maxDistSq );
		}

		m_largestDistanceFromOrigin = sqrt( maxDistSq );
		m_largestDistanceFromOriginIsUpdated = true;
	}
	return m_largestDistanceFromOrigin;
}


/*---------------------------------------------------------------
						getAllPoints
---------------------------------------------------------------*/
void  CPointsMap::getAllPoints( vector<float> &xs, vector<float> &ys, size_t decimation  ) const
{
	MRPT_START;
	ASSERT_(decimation>0);

	size_t N = x.size() / decimation;

	xs.resize(N);
	ys.resize(N);

	vector<float>::const_iterator    X,Y;
	vector<float>::iterator    		oX,oY;
	for (X=x.begin(),Y=y.begin(),oX=xs.begin(),oY=ys.begin();oX!=xs.end();X+=decimation,Y+=decimation,oX++,oY++)
	{
		*oX=*X;
		*oY=*Y;
	}
	MRPT_END;
}



/** Must return the number of data points */
size_t CPointsMap::kdtree_get_point_count() const
{
	return this->size();
}

/** Must fill out the data points in "data", such as the i'th point will be stored in (data[i][0],data[i][1]). */
void CPointsMap::kdtree_fill_point_data(ANNpointArray &data, const int nDims) const
{
	ASSERT_(nDims==2 || nDims==3)

	const size_t N = this->size();

	if (nDims==2)
	{
		const float  *x_ptr = &x[0];
		const float  *y_ptr = &y[0];
		for(size_t i=0;i<N;i++)
		{
			data[i][0]= *x_ptr++;
			data[i][1]= *y_ptr++;
		}
	}
	else
	{
		const float  *x_ptr = &x[0];
		const float  *y_ptr = &y[0];
		const float  *z_ptr = &z[0];
		for(size_t i=0;i<N;i++)
		{
			data[i][0]= *x_ptr++;
			data[i][1]= *y_ptr++;
			data[i][2]= *z_ptr++;
		}
	}
}


/*---------------------------------------------------------------
						squareDistanceToClosestCorrespondence
---------------------------------------------------------------*/
float CPointsMap::squareDistanceToClosestCorrespondence(
	float   x0,
	float   y0 ) const
{
	// Just the closest point:

#if 0
	return kdTreeClosestPoint2DsqrError(x0,y0);
#else
	// The distance to the line that interpolates the TWO closest points:
	float  x1,y1, x2,y2, d1,d2;
	kdTreeTwoClosestPoint2D(
		x0, y0, 	// The query
		x1, y1,   // Closest point #1
		x2, y2,   // Closest point #2
		d1,d2);

	ASSERT_(d2>=d1);

	// If the two points are too far, do not interpolate:
	float d12 = square(x1-x2)+square(y1-y2);
	if (d12 > 0.20f*0.20f || d12 < 0.03f*0.03f)
	{
		return square( x1-x0 ) + square( y1-y0 );
	}
	else
	{ // Interpolate
		double interp_x, interp_y;

		//math::closestFromPointToSegment(
		math::closestFromPointToLine(
			x0,y0, 	// the point
			x1,y1, x2,y2,   // The segment
			interp_x, interp_y // out
			);

		return square( interp_x-x0 ) + square( interp_y-y0 );
	}
#endif
}



/*---------------------------------------------------------------
				boundingBox
---------------------------------------------------------------*/
void CPointsMap::boundingBox(
	float &min_x,  float &max_x,
	float &min_y,  float &max_y,
	float &min_z,  float &max_z
	) const
{
	vector<float>::const_iterator xi, yi, zi;

	if (x.empty())
	{
		min_x = max_x =
		min_y = max_y =
		min_z = max_z = 0;
		return;
	}

	min_x =
	min_y =
	min_z = (std::numeric_limits<float>::max)();

	max_x =
	max_y =
	max_z = -(std::numeric_limits<float>::max)();

	for (xi=x.begin(), yi=y.begin(), zi=z.begin(); xi!=x.end();xi++,yi++,zi++)
	{
		min_x = min( min_x, *xi ); max_x = max( max_x, *xi );
		min_y = min( min_y, *yi ); max_y = max( max_y, *yi );
		min_z = min( min_z, *zi ); max_z = max( max_z, *zi );
	}
}


/*---------------------------------------------------------------
				computeMatchingWith3D
---------------------------------------------------------------*/
void  CPointsMap::computeMatchingWith3D(
    const CMetricMap						*otherMap2,
    const CPose3D							&otherMapPose,
    float									maxDistForCorrespondence,
    float									maxAngularDistForCorrespondence,
    const CPoint3D 							&angularDistPivotPoint,
    TMatchingPairList						&correspondences,
    float									&correspondencesRatio,
    float									*sumSqrDist	,
    bool									onlyKeepTheClosest,
	bool									onlyUniqueRobust) const
{
	MRPT_START;

	ASSERT_(otherMap2->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap) ));
	const CPointsMap		*otherMap = static_cast<const CPointsMap*>( otherMap2 );

	size_t					nLocalPoints = otherMap->size();
	size_t					nGlobalPoints = this->size();
	float					_sumSqrDist=0;
	size_t					_sumSqrCount = 0;
	size_t					nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

	float					local_x_min= std::numeric_limits<float>::max(), local_x_max= -std::numeric_limits<float>::max();
	float					global_x_min=std::numeric_limits<float>::max(), global_x_max= -std::numeric_limits<float>::max();
	float					local_y_min= std::numeric_limits<float>::max(), local_y_max= -std::numeric_limits<float>::max();
	float					global_y_min=std::numeric_limits<float>::max(), global_y_max= -std::numeric_limits<float>::max();
	float					local_z_min= std::numeric_limits<float>::max(), local_z_max= -std::numeric_limits<float>::max();
	float					global_z_min=std::numeric_limits<float>::max(), global_z_max= -std::numeric_limits<float>::max();

	double					maxDistForCorrespondenceSquared;

	unsigned int			globalIdx,localIdx;

	vector<float>					x_locals,y_locals,z_locals;

	float 		*x_locals_it,*y_locals_it,*z_locals_it;
	const float *x_other_it,*y_other_it,*z_other_it,*x_global_it,*y_global_it,*z_global_it;

	// No correspondences initially:
	correspondences.clear();
	correspondencesRatio = 0;

	// Hay mapa global?
	if (!nGlobalPoints) return;  // No

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	// -----------------------------------------------------------

	// Transladar y rotar ya todos los puntos locales
	x_locals.resize(nLocalPoints);
	y_locals.resize(nLocalPoints);
	z_locals.resize(nLocalPoints);

	float x_local,y_local,z_local;

	for (   localIdx=0,
		    x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&otherMap->x[0],
			y_other_it=&otherMap->y[0],
			z_other_it=&otherMap->z[0];
			localIdx<nLocalPoints;
			localIdx++)
	{
		// Translate and rotate each point in the "other" map:
		float  x_other = *x_other_it++;
		float  y_other = *y_other_it++;
		float  z_other = *z_other_it++;

		otherMapPose.composePoint( x_other,y_other,z_other,  x_local,y_local,z_local);

		*x_locals_it++ = x_local;
		*y_locals_it++ = y_local;
		*z_locals_it++ = z_local;

		// Find the bounding box:
		local_x_min = min(local_x_min,x_local);
		local_x_max = max(local_x_max,x_local);
		local_y_min = min(local_y_min,y_local);
		local_y_max = max(local_y_max,y_local);
		local_z_min = min(local_z_min,z_local);
		local_z_max = max(local_z_max,z_local);
	}

	// Find the bounding box:
	for (   globalIdx=0,
		    x_global_it=&x[0],
		    y_global_it=&y[0],
		    z_global_it=&z[0];
			globalIdx<nGlobalPoints;
			globalIdx++)
	{
		float global_x = *x_global_it++;
		float global_y = *y_global_it++;
		float global_z = *z_global_it++;

		global_x_min = min(global_x_min,global_x);
		global_x_max = max(global_x_max,global_x);
		global_y_min = min(global_y_min,global_y);
		global_y_max = max(global_y_max,global_y);
		global_z_min = min(global_z_min,global_z);
		global_z_max = max(global_z_max,global_z);
	}

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	if (local_x_min>global_x_max ||
		local_x_max<global_x_min ||
		local_y_min>global_y_max ||
		local_y_max<global_y_min) return;	// No hace falta hacer matching,
											//   porque es de CERO.

	// Loop for each point in local map:
	// --------------------------------------------------
	for ( localIdx=0,
			x_locals_it=&x_locals[0],
			y_locals_it=&y_locals[0],
			z_locals_it=&z_locals[0],
			x_other_it=&otherMap->x[0],
			y_other_it=&otherMap->y[0],
			z_other_it=&otherMap->z[0];
			localIdx<nLocalPoints;
			x_locals_it++,y_locals_it++,z_locals_it++,x_other_it++,y_other_it++,z_other_it++,localIdx++ )
	{
		// For speed-up:
		x_local = *x_locals_it;
		y_local = *y_locals_it;
		z_local = *z_locals_it;

//		bool thisLocalHasCorr = false;

		{
			// Find all the matchings in the requested distance:
			TMatchingPair		p,closestPair;


			// KD-TREE implementation

			// Use a KD-tree to look for the nearnest neighbor of:
			//   (x_local, y_local, z_local)
			// In "this" (global/reference) points map.

			p.this_idx = kdTreeClosestPoint3D(
				x_local,  y_local, z_local,  // Look closest to this guy
				p.this_x, p.this_y, p.this_z, // save here the closest match
				p.errorSquareAfterTransformation // save here the min. distance squared
				);

			// Compute max. allowed distance:
			maxDistForCorrespondenceSquared = square(
						maxAngularDistForCorrespondence * angularDistPivotPoint.distance3DTo(x_local,y_local,z_local) +
						maxDistForCorrespondence );

			// Distance below the threshold??
			if ( p.errorSquareAfterTransformation < maxDistForCorrespondenceSquared )
			{
				// Save all the correspondences??
				p.this_z = z[p.this_idx];
				p.other_idx = localIdx;
				p.other_x = *x_other_it;
				p.other_y = *y_other_it;
				p.other_z = *z_other_it;

				// save the correspondence:
				correspondences.push_back( p );

				// At least one:
				nOtherMapPointsWithCorrespondence++;

				// Accumulate the MSE:
				_sumSqrDist+= p.errorSquareAfterTransformation;
				_sumSqrCount++;
			}


		} // End of test_match
	} // For each local point

	// Additional consistency filter: "onlyKeepTheClosest" up to now
	//  led to just one correspondence for each "local map" point, but
	//  many of them may have as corresponding pair the same "global point"!!
	// -------------------------------------------------------------------------
	if (onlyUniqueRobust)
	{
		if (!onlyKeepTheClosest)  THROW_EXCEPTION("ERROR: onlyKeepTheClosest must be also set to true when onlyUniqueRobust=true.")

		vector<TMatchingPairPtr>	bestMatchForThisMap( nGlobalPoints, TMatchingPairPtr(NULL) );
		TMatchingPairList::iterator it;

		//   1) Go through all the correspondences and keep the best corresp.
		//       for each "global map" (this) point.
		for (it=correspondences.begin();it!=correspondences.end();it++)
		{
			if (!bestMatchForThisMap[it->this_idx])
			{
				bestMatchForThisMap[it->this_idx] = &(*it);
			}
			else
			{
				if ( it->errorSquareAfterTransformation < bestMatchForThisMap[it->this_idx]->errorSquareAfterTransformation )
					bestMatchForThisMap[it->this_idx] = &(*it);
			}
		}

		//   2) Go again through the list of correspondences and remove those
		//       who are not the best one for their corresponding global map.
		for (it=correspondences.begin();it!=correspondences.end();)
		{
			if ( bestMatchForThisMap[it->this_idx] != &(*it) )
			{
				// Remove & get next:
				it = correspondences.erase( it );
			}
			else // Do not remove, get next:
			{
				it++;
			}
		}
	} // end of additional consistency filer for "onlyKeepTheClosest"

	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	if (sumSqrDist)
	{
		if (_sumSqrCount)
				*sumSqrDist = _sumSqrDist / static_cast<double>(_sumSqrCount);
		else	*sumSqrDist = 0;
	}

	// The ratio of points in the other map with corrs:
	correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

	MRPT_END;
}

void CPointsMap::extractCylinder( const CPoint2D &center, const double radius, const double zmin, const double zmax, CPointsMap *outMap )
{
	outMap->clear();
	for( size_t k = 0; k < x.size(); k++ )
	{
		if( (z[k] <= zmax && z[k] >= zmin) && (center.distance2DTo( x[k],y[k] ) < radius ) )
			outMap->insertPoint( x[k], y[k], z[k] );
	}


}

/*---------------------------------------------------------------
 Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
	takenFrom The robot's pose the observation is supposed to be taken from.
	obs The observation.
 This method returns a likelihood in the range [0,1].
 ---------------------------------------------------------------*/
double	 CPointsMap::computeObservationLikelihood(
			const CObservation		*obs,
			const CPose3D				&takenFrom )
{
	// If not, this map is standalone: Compute the likelihood:
	double		ret = 0;

	 // This function depends on the observation type:
	 // -----------------------------------------------------
	 if ( obs->GetRuntimeClass() == CLASS_ID(CObservation2DRangeScan) )
	 {
		// Observation is a laser range scan:
		// -------------------------------------------
		const CObservation2DRangeScan *o = static_cast<const CObservation2DRangeScan*>(obs);

		// Build (if not done before) the points map representation of this observation:
		const CPointsMap *scanPoints = o->buildAuxPointsMap<CPointsMap>();

		float		sumSqrDist=0;

		const size_t N = scanPoints->x.size();
		if (!N || !this->size() ) return -100;

		const float *xs = &scanPoints->x[0];
		const float *ys = &scanPoints->y[0];
		const float *zs = &scanPoints->z[0];

		float	closest_x,closest_y,closest_z;
		float	closest_err;
		const float max_sqr_err = square(likelihoodOptions.max_corr_distance);
		int nPtsForAverage = 0;

		if (takenFrom.isHorizontal())
		{
			// optimized 2D version ---------------------------
			TPose2D  takenFrom2D = TPose2D(CPose2D(takenFrom));

			const float ccos = cos(takenFrom2D.phi);
			const float csin = sin(takenFrom2D.phi);

			for (size_t i=0;i<N;i+=likelihoodOptions.decimation,nPtsForAverage++)
			{
				// Transform the point from the scan reference to its global 3D position:
				const float xg = takenFrom2D.x + ccos * xs[i] - csin * ys[i];
				const float yg = takenFrom2D.y + csin * xs[i] + ccos * ys[i];

				kdTreeClosestPoint2D(
					xg,yg,  				// Look for the closest to this guy
					closest_x,closest_y, 	// save here the closest match
					closest_err				// save here the min. distance squared
					);

				// Put a limit:
				mrpt::utils::keep_min(closest_err, max_sqr_err);

				sumSqrDist+= closest_err;
			}

		}
		else
		{
			// Generic 3D version ---------------------------

			for (size_t i=0;i<N;i+=likelihoodOptions.decimation,nPtsForAverage++)
			{
				// Transform the point from the scan reference to its global 3D position:
				float xg,yg,zg;
				takenFrom.composePoint(xs[i],ys[i],zs[i], xg,yg,zg);

				kdTreeClosestPoint3D(
					xg,yg,zg,  						// Look for the closest to this guy
					closest_x,closest_y,closest_z, 	// save here the closest match
					closest_err						// save here the min. distance squared
					);

				// Put a limit:
				mrpt::utils::keep_min(closest_err, max_sqr_err);

				sumSqrDist+= closest_err;
			}
		}

		sumSqrDist /= nPtsForAverage;

		// Log-likelihood:
		ret = - sumSqrDist / likelihoodOptions.sigma_dist;
	 }

	 return ret;
	 /**/
}

/*---------------------------------------------------------------
						mark_as_modified
  ---------------------------------------------------------------*/
void CPointsMap::mark_as_modified() const
{
	m_largestDistanceFromOriginIsUpdated=false;
	kdtree_mark_as_outdated();
}


namespace mrpt
{
	namespace slam
	{
		// Tricky way to call to a library that depends on us, a sort of "run-time" linking:
		//  ptr_internal_build_points_map_from_scan2D is a functor in "mrpt-obs", set by "mrpt-maps" at its startup.
		extern void OBS_IMPEXP (*ptr_internal_build_points_map_from_scan2D)(const mrpt::slam::CObservation2DRangeScan &obs, mrpt::slam::CMetricMapPtr &out_map, const void *insertOps);
	}
}

void internal_build_points_map_from_scan2D(const mrpt::slam::CObservation2DRangeScan &obs, mrpt::slam::CMetricMapPtr &out_map, const void *insertOps)
{
	// Create on first call:
	if (out_map)
		return; // Already done!

	out_map = CSimplePointsMap::Create();

	if (insertOps)
		static_cast<CSimplePointsMap*>(out_map.pointer())->insertionOptions = *static_cast<const CPointsMap::TInsertionOptions*>(insertOps);

	out_map->insertObservation(&obs,NULL);
}

struct TAuxLoadFunctor
{
	TAuxLoadFunctor()
	{
		ptr_internal_build_points_map_from_scan2D = internal_build_points_map_from_scan2D;
	}
};

TAuxLoadFunctor  dummy_loader;  // used just to set "ptr_internal_build_points_map_from_scan2D"


