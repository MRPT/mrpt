/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "maps-precomp.h" // Precomp header

#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/utils/CTimeLogger.h>
#include <mrpt/system/os.h>
#include <mrpt/math/geometry.h>
#include <mrpt/utils/CStream.h>

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/maps/CSimplePointsMap.h>

#include <mrpt/opengl/CPointCloud.h>

// Observations:
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservation3DRangeScan.h>
#include <mrpt/obs/CObservationVelodyneScan.h>

#if MRPT_HAS_PCL
#   include <pcl/io/pcd_io.h>
#   include <pcl/point_types.h>
//#   include <pcl/registration/icp.h>
#endif

#if MRPT_HAS_SSE2
#	include <mrpt/utils/SSE_types.h>
#	include <mrpt/utils/SSE_macros.h>
#endif

#if MRPT_HAS_MATLAB
#	include <mexplus.h>
#endif

using namespace mrpt::poses;
using namespace mrpt::maps;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace std;


float mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE = 3.0f;


IMPLEMENTS_VIRTUAL_SERIALIZABLE(CPointsMap, CMetricMap,mrpt::maps)


float CPointsMap::COLOR_3DSCENE_R = 0;
float CPointsMap::COLOR_3DSCENE_G = 0;
float CPointsMap::COLOR_3DSCENE_B = 1;

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CPointsMap::CPointsMap() :
	insertionOptions(),
	likelihoodOptions(),
	x(),y(),z(),
	m_largestDistanceFromOrigin(0),
	m_heightfilter_z_min(-10),
	m_heightfilter_z_max(10),
	m_heightfilter_enabled(false)
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
					load2Dor3D_from_text_file
  Load from a text file. In each line there are a point coordinates.
	Returns false if any error occured, true elsewere.
  ---------------------------------------------------------------*/
bool  CPointsMap::load2Dor3D_from_text_file(
	const std::string &file,
	const bool is_3D)
{
	MRPT_START

	mark_as_modified();

	FILE	*f=os::fopen(file.c_str(),"rt");
	if (!f) return false;

	char		str[1024];
	char		*ptr,*ptr1,*ptr2,*ptr3;

	// Clear current map:
	this->clear();

	while (!feof(f))
	{
		// Read one string line:
		str[0] = 0;
		if (!fgets(str,sizeof(str),f)) break;

		// Find the first digit:
		ptr=str;
		while (ptr[0] && (ptr[0]==' ' || ptr[0]=='\t' || ptr[0]=='\r' || ptr[0]=='\n'))
			ptr++;

		// And try to parse it:
		float	xx = strtod(ptr,&ptr1);
		if (ptr1!=str)
		{
			float	yy = strtod(ptr1,&ptr2);
			if (ptr2!=ptr1)
			{
				if (!is_3D)
				{
					this->insertPoint(xx,yy,0);
				}
				else
				{
					float	zz = strtod(ptr2,&ptr3);
					if (ptr3!=ptr2)
					{
						this->insertPoint(xx,yy,zz);
					}
				}
			}
		}
	}

	os::fclose(f);
	return true;

	MRPT_END
}


/*---------------------------------------------------------------
  Implements the writing to a mxArray for Matlab
 ---------------------------------------------------------------*/
#if MRPT_HAS_MATLAB
// Add to implement mexplus::from template specialization
IMPLEMENTS_MEXPLUS_FROM( mrpt::maps::CPointsMap )

mxArray* CPointsMap::writeToMatlab() const
{
	MRPT_TODO("Create 3xN array xyz of points coordinates")
	const char* fields[] = {"x","y","z"};
	mexplus::MxArray map_struct( mexplus::MxArray::Struct(sizeof(fields)/sizeof(fields[0]),fields) );

	map_struct.set("x", this->x);
	map_struct.set("y", this->y);
	map_struct.set("z", this->z);
	return map_struct.release();
}
#endif


/*---------------------------------------------------------------
					getPoint
			Access to stored points coordinates:
  ---------------------------------------------------------------*/
unsigned long  CPointsMap::getPoint(size_t index,float &x,float &y) const
{
	ASSERT_BELOW_(index,this->x.size())

	x = this->x[index];
	y = this->y[index];

	return getPointWeight(index);
}
unsigned long  CPointsMap::getPoint(size_t index,float &x,float &y,float &z) const
{
	ASSERT_BELOW_(index,this->x.size())

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	return getPointWeight(index);
}
unsigned long  CPointsMap::getPoint(size_t index,double &x,double  &y) const
{
	ASSERT_BELOW_(index,this->x.size())

	x = this->x[index];
	y = this->y[index];

	return getPointWeight(index);;
}
unsigned long  CPointsMap::getPoint(size_t index,double &x,double &y,double &z) const
{
	ASSERT_BELOW_(index,this->x.size())

	x = this->x[index];
	y = this->y[index];
	z = this->z[index];

	return getPointWeight(index);;
}

/*---------------------------------------------------------------
						getPointsBuffer
 ---------------------------------------------------------------*/
void  CPointsMap::getPointsBuffer( size_t &outPointsCount, const float *&xs, const float *&ys, const float *&zs ) const
{
	outPointsCount = size();

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
	const size_t	n=size();
	vector<bool>	deletionMask(n);

	// Compute it:
	for (size_t i=0;i<n;i++)
		deletionMask[i] = ( z[i]<zMin || z[i]>zMax );

	// Perform deletion:
	applyDeletionMask(deletionMask);

	mark_as_modified();
}


/*---------------------------------------------------------------
						clipOutOfRange
 ---------------------------------------------------------------*/
void  CPointsMap::clipOutOfRange(const TPoint2D	&p, float maxRange)
{
	size_t			i,n=size();
	vector<bool>	deletionMask;

	// The deletion mask:
	deletionMask.resize(n);

	// Compute it:
	for (i=0;i<n;i++)
		deletionMask[i] = std::sqrt( square(p.x-x[i]) + square(p.y-y[i])) > maxRange;

	// Perform deletion:
	applyDeletionMask(deletionMask);

	mark_as_modified();
}

void CPointsMap::determineMatching2D(
	const mrpt::maps::CMetricMap      * otherMap2,
	const CPose2D         & otherMapPose_,
	TMatchingPairList     & correspondences,
	const TMatchingParams & params,
	TMatchingExtraResults & extraResults ) const
{
	MRPT_START

	extraResults = TMatchingExtraResults(); // Clear output

	ASSERT_ABOVE_(params.decimation_other_map_points,0)
	ASSERT_BELOW_(params.offset_other_map_points, params.decimation_other_map_points)
	ASSERT_(otherMap2->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap) ));
	const CPointsMap		*otherMap = static_cast<const CPointsMap*>( otherMap2 );

	const TPose2D otherMapPose(otherMapPose_.x(),otherMapPose_.y(),otherMapPose_.phi());

	const size_t nLocalPoints = otherMap->size();
	const size_t nGlobalPoints = this->size();
	float _sumSqrDist=0;
	size_t _sumSqrCount = 0;
	size_t nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

	float local_x_min= std::numeric_limits<float>::max(), local_x_max= -std::numeric_limits<float>::max();
	float global_x_min=std::numeric_limits<float>::max(), global_x_max= -std::numeric_limits<float>::max();
	float local_y_min= std::numeric_limits<float>::max(), local_y_max= -std::numeric_limits<float>::max();
	float global_y_min=std::numeric_limits<float>::max(), global_y_max= -std::numeric_limits<float>::max();

	double       maxDistForCorrespondenceSquared;
	float        x_local, y_local;
	unsigned int localIdx;

	const float *x_other_it,*y_other_it,*z_other_it; // *x_global_it,*y_global_it; //,*z_global_it;

	// Prepare output: no correspondences initially:
	correspondences.clear();
	correspondences.reserve(nLocalPoints);
	extraResults.correspondencesRatio = 0;

    TMatchingPairList _correspondences;
    _correspondences.reserve(nLocalPoints);

	// Hay mapa global?
	if (!nGlobalPoints) return;  // No

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	const double	sin_phi = sin(otherMapPose.phi);
	const double	cos_phi = cos(otherMapPose.phi);

	// Do matching only there is any chance of the two maps to overlap:
	// -----------------------------------------------------------

	// Translate and rotate all local points:

#if MRPT_HAS_SSE2
	// Number of 4-floats:
	size_t nPackets = nLocalPoints/4;
	if ( (nLocalPoints & 0x03)!=0) nPackets++;

	// Pad with zeros to make sure we have a number of points multiple of 4
	size_t nLocalPoints_4align = nLocalPoints;
	size_t nExtraPad = 0;
	if (0!=(nLocalPoints & 0x03))
	{
		nExtraPad = 4 - (nLocalPoints & 0x03);
		nLocalPoints_4align+=nExtraPad;
	}

	Eigen::Array<float,Eigen::Dynamic,1>  x_locals(nLocalPoints_4align), y_locals(nLocalPoints_4align);

	// We'll assume that the real allocated memory in the source buffers at least have room for a maximum
	//  of 3 more floats, and pad with zeroes there (yeah, fuck correct-constness....)
	// JLBC OCT/2016: resize() methods in maps have been modified to enforce capacities to be 4*N by design, 
	// but will leave this code here just in case (for some edge cases?)
	if ( otherMap->x.capacity()<nLocalPoints_4align ||
		 otherMap->y.capacity()<nLocalPoints_4align )
	{
		// This will happen perhaps...once in a lifetime? Anyway:
		const_cast<vector<float>*>(&otherMap->x)->reserve(nLocalPoints_4align+16);
		const_cast<vector<float>*>(&otherMap->y)->reserve(nLocalPoints_4align+16);
	}

	if (nExtraPad)
	{
		float *ptr_in_x = const_cast<float*>(&otherMap->x[0]);
		float *ptr_in_y = const_cast<float*>(&otherMap->y[0]);
		for (size_t k=nExtraPad;k; k--) {
			ptr_in_x[nLocalPoints+k]=0;
			ptr_in_y[nLocalPoints+k]=0;
		}
	}

	const __m128 cos_4val = _mm_set1_ps(cos_phi); // load 4 copies of the same value
	const __m128 sin_4val = _mm_set1_ps(sin_phi);
	const __m128 x0_4val = _mm_set1_ps(otherMapPose.x);
	const __m128 y0_4val = _mm_set1_ps(otherMapPose.y);

	// For the bounding box:
	__m128 x_mins = _mm_set1_ps( std::numeric_limits<float>::max() );
	__m128 x_maxs = _mm_set1_ps( std::numeric_limits<float>::min() );
	__m128 y_mins = x_mins;
	__m128 y_maxs = x_maxs;

	const float *ptr_in_x = &otherMap->x[0];
	const float *ptr_in_y = &otherMap->y[0];
	float *ptr_out_x    = &x_locals[0];
	float *ptr_out_y    = &y_locals[0];

	for( ; nPackets; nPackets--, ptr_in_x+=4, ptr_in_y+=4, ptr_out_x+=4, ptr_out_y+=4 )
	{
		const __m128 xs = _mm_loadu_ps(ptr_in_x); // *Unaligned* load
		const __m128 ys = _mm_loadu_ps(ptr_in_y);

		const __m128 lxs = _mm_add_ps(x0_4val, _mm_sub_ps( _mm_mul_ps(xs,cos_4val), _mm_mul_ps(ys,sin_4val) ) );
		const __m128 lys = _mm_add_ps(y0_4val, _mm_add_ps( _mm_mul_ps(xs,sin_4val), _mm_mul_ps(ys,cos_4val) ) );
		_mm_store_ps(ptr_out_x, lxs );
		_mm_store_ps(ptr_out_y, lys );

		x_mins = _mm_min_ps(x_mins,lxs);
		x_maxs = _mm_max_ps(x_maxs,lxs);
		y_mins = _mm_min_ps(y_mins,lys);
		y_maxs = _mm_max_ps(y_maxs,lys);
	}

	// Recover the min/max:
	MRPT_ALIGN16 float temp_nums[4];

	_mm_store_ps(temp_nums, x_mins); local_x_min=min(min(temp_nums[0],temp_nums[1]),min(temp_nums[2],temp_nums[3]));
	_mm_store_ps(temp_nums, y_mins); local_y_min=min(min(temp_nums[0],temp_nums[1]),min(temp_nums[2],temp_nums[3]));
	_mm_store_ps(temp_nums, x_maxs); local_x_max=max(max(temp_nums[0],temp_nums[1]),max(temp_nums[2],temp_nums[3]));
	_mm_store_ps(temp_nums, y_maxs); local_y_max=max(max(temp_nums[0],temp_nums[1]),max(temp_nums[2],temp_nums[3]));

#else
	// Non SSE2 version:
	const Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,1> > x_org( const_cast<float*>(&otherMap->x[0]),otherMap->x.size(),1 ); 
	const Eigen::Map<Eigen::Matrix<float,Eigen::Dynamic,1> > y_org( const_cast<float*>(&otherMap->y[0]),otherMap->y.size(),1 ); 

	Eigen::Array<float,Eigen::Dynamic,1>  x_locals = otherMapPose.x + cos_phi * x_org.array() - sin_phi *  y_org.array() ;
	Eigen::Array<float,Eigen::Dynamic,1>  y_locals = otherMapPose.y + sin_phi * x_org.array() + cos_phi *  y_org.array() ;

	local_x_min=x_locals.minCoeff();
	local_y_min=y_locals.minCoeff();
	local_x_max=x_locals.maxCoeff();
	local_y_max=y_locals.maxCoeff();
#endif

	// Find the bounding box:
	float global_z_min,global_z_max;
	this->boundingBox(
		global_x_min,global_x_max,
		global_y_min,global_y_max,
		global_z_min,global_z_max );

	// Only try doing a matching if there exist any chance of both maps touching/overlaping:
	if (local_x_min>global_x_max ||
		local_x_max<global_x_min ||
		local_y_min>global_y_max ||
		local_y_max<global_y_min) return;	// We know for sure there is no matching at all


	// Loop for each point in local map:
	// --------------------------------------------------
	for ( localIdx=params.offset_other_map_points,
			x_other_it=&otherMap->x[params.offset_other_map_points],
			y_other_it=&otherMap->y[params.offset_other_map_points],
			z_other_it=&otherMap->z[params.offset_other_map_points];
			localIdx<nLocalPoints;
			x_other_it+=params.decimation_other_map_points,y_other_it+=params.decimation_other_map_points,z_other_it+=params.decimation_other_map_points,localIdx+=params.decimation_other_map_points )
	{
		// For speed-up:
		x_local = x_locals[localIdx]; // *x_locals_it;
		y_local = y_locals[localIdx]; // *y_locals_it;

		// Find all the matchings in the requested distance:

		// KD-TREE implementation =================================
		// Use a KD-tree to look for the nearnest neighbor of:
		//   (x_local, y_local, z_local)
		// In "this" (global/reference) points map.

		float tentativ_err_sq;
		unsigned int tentativ_this_idx = kdTreeClosestPoint2D(
			x_local,  y_local,  // Look closest to this guy
			tentativ_err_sq // save here the min. distance squared
			);

		// Compute max. allowed distance:
		maxDistForCorrespondenceSquared = square(
			params.maxAngularDistForCorrespondence * std::sqrt( square(params.angularDistPivotPoint.x-x_local) + square(params.angularDistPivotPoint.y-y_local) ) +
					params.maxDistForCorrespondence );

		// Distance below the threshold??
		if ( tentativ_err_sq < maxDistForCorrespondenceSquared )
		{
			// Save all the correspondences:
			_correspondences.resize(_correspondences.size()+1);

			TMatchingPair & p = _correspondences.back();

			p.this_idx = tentativ_this_idx;
			p.this_x = x[tentativ_this_idx];
			p.this_y = y[tentativ_this_idx];
			p.this_z = z[tentativ_this_idx];

			p.other_idx = localIdx;
			p.other_x = *x_other_it;
			p.other_y = *y_other_it;
			p.other_z = *z_other_it;

			p.errorSquareAfterTransformation  = tentativ_err_sq;

			// At least one:
			nOtherMapPointsWithCorrespondence++;

			// Accumulate the MSE:
			_sumSqrDist+= p.errorSquareAfterTransformation;
			_sumSqrCount++;
		}

	} // For each local point

	// Additional consistency filter: "onlyKeepTheClosest" up to now
	//  led to just one correspondence for each "local map" point, but
	//  many of them may have as corresponding pair the same "global point"!!
	// -------------------------------------------------------------------------
	if (params.onlyUniqueRobust)
	{
		//if (!params.onlyKeepTheClosest)  THROW_EXCEPTION("ERROR: onlyKeepTheClosest must be also set to true when onlyUniqueRobust=true.")

		vector<TMatchingPairPtr>	bestMatchForThisMap( nGlobalPoints, TMatchingPairPtr(NULL) );
		TMatchingPairList::iterator it;

		//   1) Go through all the correspondences and keep the best corresp.
		//       for each "global map" (this) point.
		for (it=_correspondences.begin();it!=_correspondences.end();++it)
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
		for (it=_correspondences.begin();it!=_correspondences.end(); ++it)
		{
			if ( bestMatchForThisMap[it->this_idx] != &(*it) )
                correspondences.push_back( *it );   				// Add to the output
		}
	} // end of additional consistency filer for "onlyKeepTheClosest"
	else
	{
	    correspondences.swap(_correspondences);
	}

	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	if (_sumSqrCount)
			extraResults.sumSqrDist = _sumSqrDist / static_cast<double>(_sumSqrCount);
	else	extraResults.sumSqrDist = 0;

	// The ratio of points in the other map with corrs:
	extraResults.correspondencesRatio = params.decimation_other_map_points*nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

	MRPT_END
}

/*---------------------------------------------------------------
				changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointsMap::changeCoordinatesReference(const CPose2D	&newBase)
{
	const size_t N = x.size();

	const CPose3D	newBase3D(newBase);

	for (size_t i=0;i<N;i++)
		newBase3D.composePoint(
			x[i],y[i],z[i],  // In
			x[i],y[i],z[i]   // Out
		);

	mark_as_modified();
}

/*---------------------------------------------------------------
				changeCoordinatesReference
 ---------------------------------------------------------------*/
void  CPointsMap::changeCoordinatesReference(const CPose3D	&newBase)
{
	const size_t N = x.size();

	for (size_t i=0;i<N;i++)
		newBase.composePoint(
			x[i],y[i],z[i],  // In
			x[i],y[i],z[i]   // Out
		);

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
	minDistBetweenLaserPoints   ( 0.02f),
	addToExistingPointsMap      ( true),
	also_interpolate            ( false),
	disableDeletion             ( true),
	fuseWithExisting            ( false),
	isPlanarMap                 ( false),
	horizontalTolerance         ( DEG2RAD(0.05) ),
	maxDistForInterpolatePoints ( 2.0f ),
	insertInvalidPoints         ( false)
{
}

// Binary dump to/read from stream - for usage in derived classes' serialization
void CPointsMap::TInsertionOptions::writeToStream(mrpt::utils::CStream &out) const
{
	const int8_t version = 0;
	out << version;

	out
	<< minDistBetweenLaserPoints << addToExistingPointsMap << also_interpolate
	<< disableDeletion << fuseWithExisting << isPlanarMap << horizontalTolerance
	<< maxDistForInterpolatePoints << insertInvalidPoints; // v0
}

void CPointsMap::TInsertionOptions::readFromStream(mrpt::utils::CStream &in)
{
	int8_t version;
	in >> version;
	switch(version)
	{
		case 0:
		{
			in
			>> minDistBetweenLaserPoints >> addToExistingPointsMap >> also_interpolate
			>> disableDeletion >> fuseWithExisting >> isPlanarMap >> horizontalTolerance
			>> maxDistForInterpolatePoints >> insertInvalidPoints; // v0
		}
		break;
		default: MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)
	}
}


CPointsMap::TLikelihoodOptions::TLikelihoodOptions() :
    sigma_dist          ( 0.0025 ),
	max_corr_distance   ( 1.0 ),
	decimation          ( 10 )
{

}

void CPointsMap::TLikelihoodOptions::writeToStream(mrpt::utils::CStream &out) const
{
	const int8_t version = 0;
	out << version;
	out << sigma_dist << max_corr_distance << decimation;
}

void CPointsMap::TLikelihoodOptions::readFromStream(mrpt::utils::CStream &in)
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
void  CPointsMap::TInsertionOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
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

	LOADABLEOPTS_DUMP_VAR(insertInvalidPoints,bool);

	out.printf("\n");
}

void  CPointsMap::TLikelihoodOptions::dumpToTextStream(mrpt::utils::CStream	&out) const
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

	MRPT_LOAD_CONFIG_VAR(insertInvalidPoints,bool, iniFile,section);
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
	if (!genericMapParams.enableSaveAs3DObject) return;

	opengl::CPointCloudPtr  obj = opengl::CPointCloud::Create();

	obj->loadFromPointsMap(this);
	obj->setColor( CPointsMap::COLOR_3DSCENE_R, CPointsMap::COLOR_3DSCENE_G, CPointsMap::COLOR_3DSCENE_B, 1 );
	obj->setPointSize( mrpt::global_settings::POINTSMAPS_3DOBJECT_POINTSIZE );
	obj->enableColorFromZ(true);

	obj->setGradientColors( TColorf(0.0,0,0), TColorf(CPointsMap::COLOR_3DSCENE_R,CPointsMap::COLOR_3DSCENE_G,CPointsMap::COLOR_3DSCENE_B));

	outObj->insert(obj);
}

float  CPointsMap::compute3DMatchingRatio(const mrpt::maps::CMetricMap *otherMap2, const mrpt::poses::CPose3D &otherMapPose, const TMatchingRatioParams &mrp) const
{
	TMatchingPairList     correspondences;
	TMatchingParams       params;
	TMatchingExtraResults extraResults;

	params.maxDistForCorrespondence = mrp.maxDistForCorr;

	this->determineMatching3D(
		otherMap2->getAsSimplePointsMap(),
		otherMapPose,
		correspondences,
		params, extraResults);

	return extraResults.correspondencesRatio;
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
		for (X=x.begin(),Y=y.begin(),Z=z.begin();X!=x.end();++X,++Y,++Z)
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
	MRPT_START
	ASSERT_(decimation>0)

	if (decimation==1)
	{
		xs = x;
		ys = y;
	}
	else
	{
		size_t N = x.size() / decimation;

		xs.resize(N);
		ys.resize(N);

		vector<float>::const_iterator    X,Y;
		vector<float>::iterator    		oX,oY;
		for (X=x.begin(),Y=y.begin(),oX=xs.begin(),oY=ys.begin();oX!=xs.end();X+=decimation,Y+=decimation,++oX,++oY)
		{
			*oX=*X;
			*oY=*Y;
		}
	}
	MRPT_END
}


/*---------------------------------------------------------------
						squareDistanceToClosestCorrespondence
---------------------------------------------------------------*/
float CPointsMap::squareDistanceToClosestCorrespondence(
	float   x0,
	float   y0 ) const
{
	// Just the closest point:

#if 1
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
	const size_t nPoints = x.size();

	if (!m_boundingBoxIsUpdated)
	{
		if (!nPoints)
		{
			m_bb_min_x = m_bb_max_x =
			m_bb_min_y = m_bb_max_y =
			m_bb_min_z = m_bb_max_z = 0;
		}
		else
		{
#if MRPT_HAS_SSE2
			// Vectorized version: ~ 9x times faster

			// Number of 4-floats:
			size_t nPackets = nPoints/4;
			if ( (nPoints & 0x03)!=0) nPackets++;

			// Pad with zeros to make sure we have a number of points multiple of 4
			size_t nPoints_4align = nPoints;
			size_t nExtraPad = 0;
			if (0!=(nPoints & 0x03))
			{
				nExtraPad = 4 - (nPoints & 0x03);
				nPoints_4align+=nExtraPad;
			}

			// We'll assume that the real allocated memory in the source buffers at least have room for a maximum
			//  of 3 more floats, and pad with zeroes there (yeah, fuck correct-constness....)
			// JLBC OCT/2016: resize() methods in maps have been modified to enforce capacities to be 4*N by design, 
			// but will leave this code here just in case (for some edge cases?)
			if ( x.capacity()<nPoints_4align ||
				 y.capacity()<nPoints_4align ||
				 z.capacity()<nPoints_4align )
			{
				// This will happen perhaps...once in a lifetime? Anyway:
				const_cast<vector<float>*>(&x)->reserve(nPoints_4align+16);
				const_cast<vector<float>*>(&y)->reserve(nPoints_4align+16);
				const_cast<vector<float>*>(&z)->reserve(nPoints_4align+16);
			}

			if (nExtraPad)
			{
				float *ptr_in_x = const_cast<float*>(&x[0]);
				float *ptr_in_y = const_cast<float*>(&y[0]);
				float *ptr_in_z = const_cast<float*>(&z[0]);
				for (size_t k=nExtraPad;k; k--) {
					ptr_in_x[nPoints+k-1]=0;
					ptr_in_y[nPoints+k-1]=0;
					ptr_in_z[nPoints+k-1]=0;
				}
			}

			// For the bounding box:
			__m128 x_mins = _mm_set1_ps( std::numeric_limits<float>::max() );
			__m128 x_maxs = _mm_set1_ps( std::numeric_limits<float>::min() );
			__m128 y_mins = x_mins, y_maxs = x_maxs;
			__m128 z_mins = x_mins, z_maxs = x_maxs;

			const float *ptr_in_x = &this->x[0];
			const float *ptr_in_y = &this->y[0];
			const float *ptr_in_z = &this->z[0];

			for( ; nPackets; nPackets--, ptr_in_x+=4, ptr_in_y+=4, ptr_in_z+=4 )
			{
				const __m128 xs = _mm_loadu_ps(ptr_in_x); // *Unaligned* load
				x_mins = _mm_min_ps(x_mins,xs); x_maxs = _mm_max_ps(x_maxs,xs);

				const __m128 ys = _mm_loadu_ps(ptr_in_y);
				y_mins = _mm_min_ps(y_mins,ys); y_maxs = _mm_max_ps(y_maxs,ys);

				const __m128 zs = _mm_loadu_ps(ptr_in_z);
				z_mins = _mm_min_ps(z_mins,zs); z_maxs = _mm_max_ps(z_maxs,zs);
			}

			// Recover the min/max:
			MRPT_ALIGN16 float temp_nums[4];

			_mm_store_ps(temp_nums, x_mins); m_bb_min_x=min(min(temp_nums[0],temp_nums[1]),min(temp_nums[2],temp_nums[3]));
			_mm_store_ps(temp_nums, y_mins); m_bb_min_y=min(min(temp_nums[0],temp_nums[1]),min(temp_nums[2],temp_nums[3]));
			_mm_store_ps(temp_nums, z_mins); m_bb_min_z=min(min(temp_nums[0],temp_nums[1]),min(temp_nums[2],temp_nums[3]));
			_mm_store_ps(temp_nums, x_maxs); m_bb_max_x=max(max(temp_nums[0],temp_nums[1]),max(temp_nums[2],temp_nums[3]));
			_mm_store_ps(temp_nums, y_maxs); m_bb_max_y=max(max(temp_nums[0],temp_nums[1]),max(temp_nums[2],temp_nums[3]));
			_mm_store_ps(temp_nums, z_maxs); m_bb_max_z=max(max(temp_nums[0],temp_nums[1]),max(temp_nums[2],temp_nums[3]));

#else
			// Non vectorized version:
			m_bb_min_x =
			m_bb_min_y =
			m_bb_min_z = (std::numeric_limits<float>::max)();

			m_bb_max_x =
			m_bb_max_y =
			m_bb_max_z = -(std::numeric_limits<float>::max)();

			for (vector<float>::const_iterator xi=x.begin(), yi=y.begin(), zi=z.begin(); xi!=x.end();xi++,yi++,zi++)
			{
				m_bb_min_x = min( m_bb_min_x, *xi ); m_bb_max_x = max( m_bb_max_x, *xi );
				m_bb_min_y = min( m_bb_min_y, *yi ); m_bb_max_y = max( m_bb_max_y, *yi );
				m_bb_min_z = min( m_bb_min_z, *zi ); m_bb_max_z = max( m_bb_max_z, *zi );
			}
#endif

		}
		m_boundingBoxIsUpdated = true;
	}

	min_x = m_bb_min_x; max_x = m_bb_max_x;
	min_y = m_bb_min_y; max_y = m_bb_max_y;
	min_z = m_bb_min_z; max_z = m_bb_max_z;
}


/*---------------------------------------------------------------
				computeMatchingWith3D
---------------------------------------------------------------*/
void  CPointsMap::determineMatching3D(
	const mrpt::maps::CMetricMap      * otherMap2,
	const CPose3D         & otherMapPose,
	TMatchingPairList     & correspondences,
	const TMatchingParams & params,
	TMatchingExtraResults & extraResults ) const
{
	MRPT_START

	extraResults = TMatchingExtraResults();

	ASSERT_ABOVE_(params.decimation_other_map_points,0)
	ASSERT_BELOW_(params.offset_other_map_points, params.decimation_other_map_points)

	ASSERT_(otherMap2->GetRuntimeClass()->derivedFrom( CLASS_ID(CPointsMap) ));
	const CPointsMap		*otherMap = static_cast<const CPointsMap*>( otherMap2 );

	const size_t nLocalPoints = otherMap->size();
	const size_t nGlobalPoints = this->size();
	float					_sumSqrDist=0;
	size_t					_sumSqrCount = 0;
	size_t					nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

	float local_x_min= std::numeric_limits<float>::max(), local_x_max= -std::numeric_limits<float>::max();
	float local_y_min= std::numeric_limits<float>::max(), local_y_max= -std::numeric_limits<float>::max();
	float local_z_min= std::numeric_limits<float>::max(), local_z_max= -std::numeric_limits<float>::max();

	double					maxDistForCorrespondenceSquared;


	// Prepare output: no correspondences initially:
	correspondences.clear();
	correspondences.reserve(nLocalPoints);

    TMatchingPairList _correspondences;
	_correspondences.reserve(nLocalPoints);

	// Hay mapa global?
	if (!nGlobalPoints) return;  // No

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	// -----------------------------------------------------------

	// Transladar y rotar ya todos los puntos locales
	vector<float> x_locals(nLocalPoints), y_locals(nLocalPoints), z_locals(nLocalPoints);

	for (unsigned int localIdx=params.offset_other_map_points;localIdx<nLocalPoints;localIdx+=params.decimation_other_map_points)
	{
		float x_local,y_local,z_local;
		otherMapPose.composePoint(
			otherMap->x[localIdx], otherMap->y[localIdx], otherMap->z[localIdx],
			x_local,y_local,z_local );

		x_locals[localIdx] = x_local;
		y_locals[localIdx] = y_local;
		z_locals[localIdx] = z_local;

		// Find the bounding box:
		local_x_min = min(local_x_min,x_local);
		local_x_max = max(local_x_max,x_local);
		local_y_min = min(local_y_min,y_local);
		local_y_max = max(local_y_max,y_local);
		local_z_min = min(local_z_min,z_local);
		local_z_max = max(local_z_max,z_local);
	}

	// Find the bounding box:
	float global_x_min, global_x_max, global_y_min, global_y_max, global_z_min, global_z_max;
	this->boundingBox(
		global_x_min,global_x_max,
		global_y_min,global_y_max,
		global_z_min,global_z_max );

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	if (local_x_min>global_x_max ||
		local_x_max<global_x_min ||
		local_y_min>global_y_max ||
		local_y_max<global_y_min) return;	// No hace falta hacer matching,
											//   porque es de CERO.

	// Loop for each point in local map:
	// --------------------------------------------------
	for (unsigned int localIdx=params.offset_other_map_points; localIdx<nLocalPoints; localIdx+=params.decimation_other_map_points)
	{
		// For speed-up:
		const float x_local = x_locals[localIdx];
		const float y_local = y_locals[localIdx];
		const float z_local = z_locals[localIdx];

		{
			// KD-TREE implementation
			// Use a KD-tree to look for the nearnest neighbor of:
			//   (x_local, y_local, z_local)
			// In "this" (global/reference) points map.

			float tentativ_err_sq;
			const unsigned int tentativ_this_idx = kdTreeClosestPoint3D(
				x_local,  y_local, z_local,  // Look closest to this guy
				tentativ_err_sq // save here the min. distance squared
				);

			// Compute max. allowed distance:
			maxDistForCorrespondenceSquared = square(
						params.maxAngularDistForCorrespondence * params.angularDistPivotPoint.distanceTo(TPoint3D(x_local,y_local,z_local)) +
						params.maxDistForCorrespondence );

			// Distance below the threshold??
			if ( tentativ_err_sq < maxDistForCorrespondenceSquared )
			{
				// Save all the correspondences:
				_correspondences.resize(_correspondences.size()+1);

				TMatchingPair & p = _correspondences.back();

				p.this_idx = tentativ_this_idx;
				p.this_x = x[tentativ_this_idx];
				p.this_y = y[tentativ_this_idx];
				p.this_z = z[tentativ_this_idx];

				p.other_idx = localIdx;
				p.other_x = otherMap->x[localIdx];
				p.other_y = otherMap->y[localIdx];
				p.other_z = otherMap->z[localIdx];

				p.errorSquareAfterTransformation  = tentativ_err_sq;

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
	if (params.onlyUniqueRobust)
	{
		if (!params.onlyKeepTheClosest)  THROW_EXCEPTION("ERROR: onlyKeepTheClosest must be also set to true when onlyUniqueRobust=true.")

		vector<TMatchingPairPtr>	bestMatchForThisMap( nGlobalPoints, TMatchingPairPtr(NULL) );
		TMatchingPairList::iterator it;

		//   1) Go through all the correspondences and keep the best corresp.
		//       for each "global map" (this) point.
		for (it=_correspondences.begin();it!=_correspondences.end();++it)
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
		for (it=_correspondences.begin();it!=_correspondences.end(); ++it)
		{
			if ( bestMatchForThisMap[it->this_idx] != &(*it) )
				correspondences.push_back(*it); 				// Add to the output
		}
	} // end of additional consistency filer for "onlyKeepTheClosest"
	else
	{
	    correspondences.swap(_correspondences);
	}

	// If requested, copy sum of squared distances to output pointer:
	// -------------------------------------------------------------------
	extraResults.sumSqrDist = (_sumSqrCount) ? _sumSqrDist / static_cast<double>(_sumSqrCount) : 0;
	extraResults.correspondencesRatio = params.decimation_other_map_points*nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

	MRPT_END
}

/*---------------------------------------------------------------
				extractCylinder
---------------------------------------------------------------*/
void CPointsMap::extractCylinder( const TPoint2D &center, const double radius, const double zmin, const double zmax, CPointsMap *outMap )
{
	outMap->clear();
	for( size_t k = 0; k < x.size(); k++ )
	{
		if( (z[k] <= zmax && z[k] >= zmin) && ( sqrt(square(center.x-x[k])+square(center.y-y[k])) < radius ) )
			outMap->insertPoint( x[k], y[k], z[k] );
	}
}

/*---------------------------------------------------------------
				extractPoints
---------------------------------------------------------------*/
void CPointsMap::extractPoints( const TPoint3D &corner1, const TPoint3D &corner2, CPointsMap *outMap, const double &R, const double &G, const double &B )
{
	outMap->clear();
	double minX,maxX,minY,maxY,minZ,maxZ;
	minX = min(corner1.x,corner2.x);    maxX = max(corner1.x,corner2.x);
	minY = min(corner1.y,corner2.y);    maxY = max(corner1.y,corner2.y);
	minZ = min(corner1.z,corner2.z);    maxZ = max(corner1.z,corner2.z);
	for( size_t k = 0; k < x.size(); k++ )
	{
		if( (x[k] >= minX && x[k] <= maxX) &&
            (y[k] >= minY && y[k] <= maxY) &&
            (z[k] >= minZ && z[k] <= maxZ) )
			outMap->insertPoint( x[k], y[k], z[k], R, G, B );
	}
}

/*---------------------------------------------------------------
				compute3DDistanceToMesh
---------------------------------------------------------------*/
void CPointsMap::compute3DDistanceToMesh(
    const mrpt::maps::CMetricMap						*otherMap2,
	const CPose3D							&otherMapPose,
	float									maxDistForCorrespondence,
	TMatchingPairList                       &correspondences,
	float           						&correspondencesRatio )
{
    MRPT_START
    MRPT_UNUSED_PARAM(otherMapPose);

    const CPointsMap		*otherMap = static_cast<const CPointsMap*>( otherMap2 );

	const size_t nLocalPoints   = otherMap->size();
	const size_t nGlobalPoints  = this->size();
	size_t nOtherMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

    // Prepare output: no correspondences initially:
    correspondences.clear();
	correspondences.reserve(nLocalPoints);
	correspondencesRatio = 0;

    // aux correspondence vector
    TMatchingPairList _correspondences;
    _correspondences.reserve(nLocalPoints);

	// Hay mapa global?
	if (!nGlobalPoints) return;  // No

	// Hay mapa local?
	if (!nLocalPoints)  return;  // No

	// we'll assume by now both reference systems are the same
    float local_x_min, local_x_max, local_y_min, local_y_max, local_z_min, local_z_max;
    otherMap->boundingBox(
        local_x_min, local_x_max,
        local_y_min, local_y_max,
        local_z_min, local_z_max );

	// Find the bounding box:
	float global_x_min, global_x_max, global_y_min, global_y_max, global_z_min, global_z_max;
	this->boundingBox(
		global_x_min,global_x_max,
		global_y_min,global_y_max,
		global_z_min,global_z_max );

	// Solo hacer matching si existe alguna posibilidad de que
	//  los dos mapas se toquen:
	if (local_x_min>global_x_max ||
		local_x_max<global_x_min ||
		local_y_min>global_y_max ||
		local_y_max<global_y_min) return;	// No hace falta hacer matching,
											//   porque es de CERO.

    std::vector< std::vector<size_t> > vIdx;

	// Loop for each point in local map:
	// --------------------------------------------------
	std::vector<float> outX,outY,outZ,tentativeErrSq;
	std::vector<size_t> outIdx;
	for (unsigned int localIdx = 0; localIdx < nLocalPoints; ++localIdx )
	{
		// For speed-up:
		const float x_local = otherMap->x[localIdx];
		const float y_local = otherMap->y[localIdx];
		const float z_local = otherMap->z[localIdx];

		{
			// KD-TREE implementation
			// Use a KD-tree to look for the nearnest neighbor of:
			//   (x_local, y_local, z_local)
			// In "this" (global/reference) points map.
			kdTreeNClosestPoint3DWithIdx(
				x_local,  y_local, z_local,     // Look closest to this guy
				3,                              // get the three closest points
				outX,outY,outZ,                 // output vectors
				outIdx,                         // output indexes
				tentativeErrSq                  // save here the min. distance squared
				);

            // get the centroid
            const float mX = (outX[0]+outX[1]+outX[2])/3.0;
            const float mY = (outY[0]+outY[1]+outY[2])/3.0;
            const float mZ = (outZ[0]+outZ[1]+outZ[2])/3.0;

            const float distanceForThisPoint = fabs(mrpt::math::distance(TPoint3D(x_local,y_local,z_local),TPoint3D(mX,mY,mZ)));

			// Distance below the threshold??
			if ( distanceForThisPoint < maxDistForCorrespondence )
			{
				// Save all the correspondences:
				_correspondences.resize(_correspondences.size()+1);

				TMatchingPair & p = _correspondences.back();

				p.this_idx = nOtherMapPointsWithCorrespondence++; // insert a consecutive index here
				p.this_x = mX;
				p.this_y = mY;
				p.this_z = mZ;

				p.other_idx = localIdx;
				p.other_x = otherMap->x[localIdx];
				p.other_y = otherMap->y[localIdx];
				p.other_z = otherMap->z[localIdx];

				p.errorSquareAfterTransformation  = distanceForThisPoint;

                // save the indexes
                std::sort(outIdx.begin(),outIdx.end());
				vIdx.push_back(outIdx);
			}
		} // End of test_match
	} // For each local point

	// Additional consistency filter: "onlyKeepTheClosest" up to now
	//  led to just one correspondence for each "local map" point, but
	//  many of them may have as corresponding pair the same "global point"!!
	// -------------------------------------------------------------------------
    std::map<size_t, std::map<size_t, std::map<size_t, pair<size_t,float> > > > best; // 3D associative map
    TMatchingPairList::iterator it;
    for (it = _correspondences.begin(); it != _correspondences.end(); ++it)
    {
        const size_t i0 = vIdx[it->this_idx][0];
        const size_t i1 = vIdx[it->this_idx][1];
        const size_t i2 = vIdx[it->this_idx][2];

        if( best.find(i0) != best.end() &&
            best[i0].find(i1) != best[i0].end() &&
            best[i0][i1].find(i2) != best[i0][i1].end() )   // if there is a match, check if it is better
        {
            if( best[i0][i1][i2].second > it->errorSquareAfterTransformation )
            {
                best[i0][i1][i2].first  = it->this_idx;
                best[i0][i1][i2].second = it->errorSquareAfterTransformation;
            }
        }
        else    // if there is no match
        {
            best[i0][i1][i2].first  = it->this_idx;
            best[i0][i1][i2].second = it->errorSquareAfterTransformation;
        }
    } // end it correspondences

    for (it = _correspondences.begin(); it != _correspondences.end(); ++it)
    {
        const size_t i0 = vIdx[it->this_idx][0];
        const size_t i1 = vIdx[it->this_idx][1];
        const size_t i2 = vIdx[it->this_idx][2];

        if( best[i0][i1][i2].first == it->this_idx )
            correspondences.push_back(*it);
    }

	// The ratio of points in the other map with corrs:
	correspondencesRatio = nOtherMapPointsWithCorrespondence / static_cast<float>(nLocalPoints);

    MRPT_END
}

/*---------------------------------------------------------------
 Computes the likelihood that a given observation was taken from a given pose in the world being modeled with this map.
	takenFrom The robot's pose the observation is supposed to be taken from.
	obs The observation.
 This method returns a likelihood in the range [0,1].
 ---------------------------------------------------------------*/
double	 CPointsMap::internal_computeObservationLikelihood(
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


namespace mrpt
{
	namespace obs
	{
		// Tricky way to call to a library that depends on us, a sort of "run-time" linking:
		//  ptr_internal_build_points_map_from_scan2D is a functor in "mrpt-obs", set by "mrpt-maps" at its startup.
		extern void OBS_IMPEXP (*ptr_internal_build_points_map_from_scan2D)(const mrpt::obs::CObservation2DRangeScan &obs, mrpt::maps::CMetricMapPtr &out_map, const void *insertOps);
	}
}

void internal_build_points_map_from_scan2D(const mrpt::obs::CObservation2DRangeScan &obs, mrpt::maps::CMetricMapPtr &out_map, const void *insertOps)
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
		mrpt::obs::ptr_internal_build_points_map_from_scan2D = internal_build_points_map_from_scan2D;
	}
};

TAuxLoadFunctor  dummy_loader;  // used just to set "ptr_internal_build_points_map_from_scan2D"




// ================================ PLY files import & export virtual methods ================================

/** In a base class, will be called after PLY_import_set_vertex_count() once for each loaded point.
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CPointsMap::PLY_import_set_vertex(const size_t idx, const mrpt::math::TPoint3Df &pt, const mrpt::utils::TColorf *pt_color)
{
	MRPT_UNUSED_PARAM(pt_color);
	this->setPoint(idx,pt.x,pt.y,pt.z);
}

/** In a base class, return the number of vertices */
size_t CPointsMap::PLY_export_get_vertex_count() const
{
	return this->size();
}

/** In a base class, will be called after PLY_export_get_vertex_count() once for each exported point.
  *  \param pt_color Will be NULL if the loaded file does not provide color info.
  */
void CPointsMap::PLY_export_get_vertex(
	const size_t idx,
	mrpt::math::TPoint3Df &pt,
	bool &pt_has_color,
	mrpt::utils::TColorf &pt_color) const
{
	MRPT_UNUSED_PARAM(pt_color);
	pt_has_color=false;

	pt.x = x[idx];
	pt.y = y[idx];
	pt.z = z[idx];
}

// Generic implementation (a more optimized one should exist in derived classes):
void  CPointsMap::addFrom(const CPointsMap &anotherMap)
{
	const size_t nThis = this->size();
	const size_t nOther = anotherMap.size();

	const size_t nTot = nThis+nOther;

	this->resize(nTot);

	for (size_t i=0,j=nThis;i<nOther;i++, j++)
	{
		this->x[j]=anotherMap.x[i];
		this->y[j]=anotherMap.y[i];
		this->z[j]=anotherMap.z[i];
	}

	// Also copy other data fields (color, ...)
	addFrom_classSpecific(anotherMap,nThis);

	mark_as_modified();
}

/** Save the point cloud as a PCL PCD file, in either ASCII or binary format \return false on any error */
bool CPointsMap::savePCDFile(const std::string &filename, bool save_as_binary) const
{
#if MRPT_HAS_PCL
	pcl::PointCloud<pcl::PointXYZ> cloud;
	this->getPCLPointCloud(cloud);

	return 0 == pcl::io::savePCDFile(filename, cloud, save_as_binary);

#else
	MRPT_UNUSED_PARAM(filename);
	MRPT_UNUSED_PARAM(save_as_binary);
	THROW_EXCEPTION("Operation not available: MRPT was built without PCL")
#endif
}

/** Load the point cloud from a PCL PCD file (requires MRPT built against PCL) \return false on any error */
bool CPointsMap::loadPCDFile(const std::string &filename)
{
#if MRPT_HAS_PCL
	pcl::PointCloud<pcl::PointXYZ> cloud;
	if (0!=pcl::io::loadPCDFile(filename,cloud))
		return false;

	this->getPCLPointCloud(cloud);

	return true;
#else
	MRPT_UNUSED_PARAM(filename);
	THROW_EXCEPTION("Operation not available: MRPT was built without PCL")
#endif
}



/*---------------------------------------------------------------
						applyDeletionMask
 ---------------------------------------------------------------*/
void  CPointsMap::applyDeletionMask( const std::vector<bool> &mask )
{
	ASSERT_EQUAL_( size(), mask.size() )

	// Remove marked points:
	const size_t n = mask.size();
	vector<float> Pt;
	size_t i,j;
	for (i=0,j=0;i<n;i++)
	{
		if (!mask[i])
		{
			// Pt[j] <---- Pt[i]
			this->getPointAllFieldsFast(i,Pt);
			this->setPointAllFieldsFast(j++,Pt);
		}
	}

	// Set new correct size:
	this->resize(j);

	mark_as_modified();
}

/*---------------------------------------------------------------
					insertAnotherMap
 ---------------------------------------------------------------*/
void  CPointsMap::insertAnotherMap(
	const CPointsMap	*otherMap,
	const CPose3D		&otherPose)
{
	const size_t N_this = size();
	const size_t N_other = otherMap->size();

	// Set the new size:
	this->resize( N_this + N_other );

	mrpt::math::TPoint3Df pt;
	size_t src,dst;
	for (src=0, dst=N_this; src<N_other; src++, dst++)
	{
		// Load the next point:
		otherMap->getPointFast(src,pt.x,pt.y,pt.z);

		// Translation:
		double gx,gy,gz;
		otherPose.composePoint(pt.x,pt.y,pt.z,  gx,gy,gz);

		// Add to this map:
		this->setPointFast(dst,gx,gy,gz);
	}

	// Also copy other data fields (color, ...)
	addFrom_classSpecific(*otherMap, N_this);

	mark_as_modified();
}


/** Helper method for ::copyFrom() */
void  CPointsMap::base_copyFrom(const CPointsMap &obj)
{
	MRPT_START

	if (this==&obj)
		return;

	x = obj.x;
	y = obj.y;
	z = obj.z;

	m_largestDistanceFromOriginIsUpdated = obj.m_largestDistanceFromOriginIsUpdated;
	m_largestDistanceFromOrigin = obj.m_largestDistanceFromOrigin;

	// Fill missing fields (R,G,B,min_dist) with default values.
	this->resize(x.size());

	kdtree_mark_as_outdated();

	MRPT_END
}


/*---------------------------------------------------------------
					internal_insertObservation

  Insert the observation information into this map.
 ---------------------------------------------------------------*/
bool  CPointsMap::internal_insertObservation(
	const CObservation	*obs,
	const CPose3D *robotPose)
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

	if (IS_CLASS(obs,CObservation2DRangeScan))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservation2DRangeScan
		 ********************************************************************/
		mark_as_modified();

		const CObservation2DRangeScan *o = static_cast<const CObservation2DRangeScan *>(obs);
		// Insert only HORIZONTAL scans??
		bool	reallyInsertIt;

		if (insertionOptions.isPlanarMap)
			 reallyInsertIt = o->isPlanarScan( insertionOptions.horizontalTolerance );
		else reallyInsertIt = true;

		if (reallyInsertIt)
		{
			std::vector<bool>	checkForDeletion;

			// 1) Fuse into the points map or add directly?
			// ----------------------------------------------
			if (insertionOptions.fuseWithExisting)
			{
				CSimplePointsMap	auxMap;
				// Fuse:
				auxMap.insertionOptions = insertionOptions;
				auxMap.insertionOptions.addToExistingPointsMap = false;

				auxMap.loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D			// The robot pose
					);

				fuseWith(	&auxMap,					// Fuse with this map
							insertionOptions.minDistBetweenLaserPoints,	// Min dist.
							&checkForDeletion		// Set to "false" if a point in "map" has been fused.
							);

				if (! insertionOptions.disableDeletion )
				{
					// 2) Delete points in newly added free
					//      region, thus dynamic areas:
					// --------------------------------------
					// Load scan as a polygon:
					CPolygon			pol;
					const float			*xs,*ys,*zs;
					size_t n;
					auxMap.getPointsBuffer( n, xs,ys,zs);
					pol.setAllVertices( n, xs, ys );

					// Check for deletion of points in "map"
					n = size();
					for (size_t i=0;i<n;i++)
					{
						if ( checkForDeletion[i] )		// Default to true, unless a fused point, which must be kept.
						{
							float x,y;
							getPoint(i,x,y);
							if ( !pol.PointIntoPolygon( x,y) )
								checkForDeletion[i] = false;	// Out of polygon, don't delete
						}
					}

					// Create a new points list just with non-deleted points.
					// ----------------------------------------------------------
					applyDeletionMask( checkForDeletion );
				}
			}
			else
			{
				// Don't fuse: Simply add
				insertionOptions.addToExistingPointsMap = true;
				loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D				// The robot pose
					);

				// Don't build this vector if is not used later!
//				if (!insertionOptions.disableDeletion)
//				{
//					const size_t n = size();
//					checkForDeletion.resize(n);
//					for (size_t i=0;i<n;i++) checkForDeletion[i] = true;
//				}
			}


			return true;
		}
		// A planar map and a non-horizontal scan.
		else return false;
	}
	else
	if (IS_CLASS(obs,CObservation3DRangeScan))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservation3DRangeScan
		 ********************************************************************/
		mark_as_modified();

		const CObservation3DRangeScan *o = static_cast<const CObservation3DRangeScan *>(obs);
		// Insert only HORIZONTAL scans??
		bool	reallyInsertIt;

		if (insertionOptions.isPlanarMap)
			 reallyInsertIt = false; // Don't insert 3D range observation into planar map
		else reallyInsertIt = true;

		if (reallyInsertIt)
		{
			// 1) Fuse into the points map or add directly?
			// ----------------------------------------------
			if (insertionOptions.fuseWithExisting)
			{
				// Fuse:
				CSimplePointsMap	auxMap;
				auxMap.insertionOptions = insertionOptions;
				auxMap.insertionOptions.addToExistingPointsMap = false;

				auxMap.loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D			// The robot pose
					);

				fuseWith(	&auxMap,					// Fuse with this map
							insertionOptions.minDistBetweenLaserPoints,	// Min dist.
							NULL			// rather than &checkForDeletion which we don't need for 3D observations
							);
			}
			else
			{
				// Don't fuse: Simply add
				insertionOptions.addToExistingPointsMap = true;
				loadFromRangeScan(
					*o,						// The laser range scan observation
					&robotPose3D			// The robot pose
					);
			}

			// This could be implemented to check whether existing points fall into empty-space 3D polygon
			// but performance for standard Swissranger scans (176*144 points) may be too sluggish?
			//if (! insertionOptions.disableDeletion )   {
			// ....
			// }
			// JL -> Nope, it's ok like that ;-)

			return true;
		}
		// A planar map and a non-horizontal scan.
		else return false;
	}
	else
	if ( IS_CLASS(obs,CObservationRange))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationRange  (IRs, Sonars, etc.)
		 ********************************************************************/
		mark_as_modified();

		const CObservationRange* o = static_cast<const CObservationRange*>(obs);

		const double aper_2 = 0.5*o->sensorConeApperture;

		this->reserve( this->size() + o->sensedData.size()*30 );  // faster push_back's.

		for (CObservationRange::const_iterator it=o->begin();it!=o->end();++it)
		{
			const CPose3D sensorPose = robotPose3D + CPose3D(it->sensorPose);
			const double rang = it->sensedDistance;

			if (rang<=0 || rang<o->minSensorDistance || rang>o->maxSensorDistance)
				continue;

			// Insert a few points with a given maximum separation between them:
			const double arc_len = o->sensorConeApperture*rang;
			const unsigned int nSteps = round(1+arc_len/0.05);
			const double Aa = o->sensorConeApperture/double(nSteps);
			TPoint3D loc, glob;

			for (double a1=-aper_2;a1<aper_2;a1+=Aa)
			{
				for (double a2=-aper_2;a2<aper_2;a2+=Aa)
				{
					loc.x = cos(a1)*cos(a2)*rang;
					loc.y = cos(a1)*sin(a2)*rang;
					loc.z = sin(a1)*rang;
					sensorPose.composePoint(loc,glob);

					this->insertPointFast(glob.x, glob.y, glob.z);
				}
			}
		}
		return true;
	}
	else
	if (IS_CLASS(obs,CObservationVelodyneScan))
	{
		/********************************************************************
					OBSERVATION TYPE: CObservationVelodyneScan
		 ********************************************************************/
		mark_as_modified();

		const CObservationVelodyneScan *o = static_cast<const CObservationVelodyneScan *>(obs);

		if (insertionOptions.fuseWithExisting) {
			// Fuse:
			CSimplePointsMap	auxMap;
			auxMap.insertionOptions = insertionOptions;
			auxMap.insertionOptions.addToExistingPointsMap = false;
			auxMap.loadFromVelodyneScan(*o,&robotPose3D);
			fuseWith(&auxMap, insertionOptions.minDistBetweenLaserPoints,  NULL /* rather than &checkForDeletion which we don't need for 3D observations */ );
		}
		else {
			// Don't fuse: Simply add
			insertionOptions.addToExistingPointsMap = true;
			loadFromVelodyneScan(*o,&robotPose3D);
		}
		return true;
	}
	else
	{
		/********************************************************************
					OBSERVATION TYPE: Unknown
		********************************************************************/
		return false;
	}

	MRPT_END
}

/*---------------------------------------------------------------
Insert the contents of another map into this one, fusing the previous content with the new one.
 This means that points very close to existing ones will be "fused", rather than "added". This prevents
 the unbounded increase in size of these class of maps.
 ---------------------------------------------------------------*/
void  CPointsMap::fuseWith(
	CPointsMap			*otherMap,
	float				minDistForFuse,
	std::vector<bool>	*notFusedPoints)
{
	TMatchingPairList	correspondences;
	TPoint3D			a,b;
	const CPose2D		nullPose(0,0,0);

	mark_as_modified();

	//const size_t nThis  =     this->size();
	const size_t nOther = otherMap->size();

	// Find correspondences between this map and the other one:
	// ------------------------------------------------------------
	TMatchingParams params;
	TMatchingExtraResults extraResults;

	params.maxAngularDistForCorrespondence = 0;
	params.maxDistForCorrespondence = minDistForFuse;

	determineMatching2D(
		otherMap,// The other map
		nullPose,	// The other map's pose
		correspondences,
		params, extraResults);

	// Initially, all set to "true" -> "not fused".
	if (notFusedPoints)
	{
		notFusedPoints->clear();
		notFusedPoints->reserve( x.size() + nOther );
		notFusedPoints->resize( x.size(), true );
	}

	// Speeds-up possible memory reallocations:
	reserve( x.size() + nOther );

	// Merge matched points from both maps:
	//  AND add new points which have been not matched:
	// -------------------------------------------------
	for (size_t i=0;i<nOther;i++)
	{
		const unsigned long	w_a = otherMap->getPoint(i,a);	// Get "local" point into "a"

		// Find closest correspondence of "a":
		int			closestCorr = -1;
		float		minDist	= std::numeric_limits<float>::max();
		for (TMatchingPairList::const_iterator corrsIt = correspondences.begin(); corrsIt!=correspondences.end(); ++corrsIt)
		{
			if (corrsIt->other_idx==i)
			{
				float	dist = square( corrsIt->other_x - corrsIt->this_x ) +
							   square( corrsIt->other_y - corrsIt->this_y ) +
							   square( corrsIt->other_z - corrsIt->this_z );
				if (dist<minDist)
				{
					minDist = dist;
					closestCorr = corrsIt->this_idx;
				}
			}
		} // End of for each correspondence...

		if (closestCorr!=-1)
		{	// Merge:		FUSION
			unsigned long w_b = getPoint(closestCorr,b);

			ASSERT_((w_a+w_b)>0);

			const float F = 1.0f/(w_a+w_b);

			x[closestCorr]=F*(w_a*a.x+w_b*b.x);
			y[closestCorr]=F*(w_a*a.y+w_b*b.y);
			z[closestCorr]=F*(w_a*a.z+w_b*b.z);

			this->setPointWeight(closestCorr,w_a+w_b);

			// Append to fused points list
			if (notFusedPoints)
				(*notFusedPoints)[closestCorr] = false;
		}
		else
		{	// New point:	ADDITION
			this->insertPointFast(a.x,a.y,a.z);
			if (notFusedPoints)
				(*notFusedPoints).push_back(false);
		}
	}
}

void CPointsMap::loadFromVelodyneScan(
	const mrpt::obs::CObservationVelodyneScan & scan,
	const mrpt::poses::CPose3D				  *robotPose)
{
	ASSERT_EQUAL_(scan.point_cloud.x.size(),scan.point_cloud.y.size());
	ASSERT_EQUAL_(scan.point_cloud.x.size(),scan.point_cloud.z.size());
	ASSERT_EQUAL_(scan.point_cloud.x.size(),scan.point_cloud.intensity.size());

	if (scan.point_cloud.x.empty())
		return;

	this->mark_as_modified();

	// Insert vs. load and replace:
	if (!insertionOptions.addToExistingPointsMap)
		resize(0); // Resize to 0 instead of clear() so the std::vector<> memory is not actually deallocated and can be reused.

	// Alloc space:
	const size_t nOldPtsCount = this->size();
	const size_t nScanPts = scan.point_cloud.size();
	const size_t nNewPtsCount = nOldPtsCount + nScanPts;
	this->resize(nNewPtsCount);

	const float K = 1.0f / 255;  // Intensity scale.

	// global 3D pose:
	CPose3D sensorGlobalPose;
	if (robotPose) 
	      sensorGlobalPose = *robotPose + scan.sensorPose;
	else  sensorGlobalPose = scan.sensorPose;

	mrpt::math::CMatrixDouble44 HM;
	sensorGlobalPose.getHomogeneousMatrix(HM);

	const double m00 = HM.get_unsafe(0,0), m01 = HM.get_unsafe(0,1), m02 = HM.get_unsafe(0,2), m03 = HM.get_unsafe(0,3);
	const double m10 = HM.get_unsafe(1,0), m11 = HM.get_unsafe(1,1), m12 = HM.get_unsafe(1,2), m13 = HM.get_unsafe(1,3);
	const double m20 = HM.get_unsafe(2,0), m21 = HM.get_unsafe(2,1), m22 = HM.get_unsafe(2,2), m23 = HM.get_unsafe(2,3);

	// Copy points:
	for (size_t i=0;i<nScanPts;i++)
	{
		const float inten = scan.point_cloud.intensity[i] * K;
		const double lx = scan.point_cloud.x[i];
		const double ly = scan.point_cloud.y[i];
		const double lz = scan.point_cloud.z[i];

		const double gx = m00*lx + m01*ly + m02*lz + m03;
		const double gy = m10*lx + m11*ly + m12*lz + m13;
		const double gz = m20*lx + m21*ly + m22*lz + m23;

		this->setPoint(nOldPtsCount+i,
			gx,gy,gz,  // XYZ
			inten,inten,inten // RGB
			);
	}
}

