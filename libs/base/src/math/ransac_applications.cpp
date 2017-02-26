/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers


#include <mrpt/math/ransac_applications.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


/*---------------------------------------------------------------
		Aux. functions needed by ransac_detect_3D_planes
 ---------------------------------------------------------------*/
namespace mrpt
{
	namespace math
	{
		template <typename T>
		void  ransac3Dplane_fit(
			const CMatrixTemplateNumeric<T> &allData,
			const vector_size_t  &useIndices,
			vector< CMatrixTemplateNumeric<T> > &fitModels )
		{
			ASSERT_(useIndices.size()==3);

			TPoint3D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]),allData(2,useIndices[0]) );
			TPoint3D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]),allData(2,useIndices[1]) );
			TPoint3D  p3( allData(0,useIndices[2]),allData(1,useIndices[2]),allData(2,useIndices[2]) );

			try
			{
				TPlane  plane( p1,p2,p3 );
				fitModels.resize(1);
				CMatrixTemplateNumeric<T> &M = fitModels[0];

				M.setSize(1,4);
				for (size_t i=0;i<4;i++)
					M(0,i)=T(plane.coefs[i]);
			}
			catch(exception &)
			{
				fitModels.clear();
				return;
			}
		}


		template <typename T>
		void ransac3Dplane_distance(
			const CMatrixTemplateNumeric<T> &allData,
			const vector< CMatrixTemplateNumeric<T> > & testModels,
			const T distanceThreshold,
			unsigned int & out_bestModelIndex,
			vector_size_t & out_inlierIndices )
		{
			ASSERT_( testModels.size()==1 )
			out_bestModelIndex = 0;
			const CMatrixTemplateNumeric<T> &M = testModels[0];

			ASSERT_( size(M,1)==1 && size(M,2)==4 )

			TPlane  plane;
			plane.coefs[0] = M(0,0);
			plane.coefs[1] = M(0,1);
			plane.coefs[2] = M(0,2);
			plane.coefs[3] = M(0,3);

			const size_t N = size(allData,2);
			out_inlierIndices.clear();
			out_inlierIndices.reserve(100);
			for (size_t i=0;i<N;i++)
			{
				const double d = plane.distance( TPoint3D( allData.get_unsafe(0,i),allData.get_unsafe(1,i),allData.get_unsafe(2,i) ) );
				if (d<distanceThreshold)
					out_inlierIndices.push_back(i);
			}
		}

		/** Return "true" if the selected points are a degenerate (invalid) case.
		  */
		template <typename T>
		bool ransac3Dplane_degenerate(
			const CMatrixTemplateNumeric<T> &allData,
			const mrpt::vector_size_t &useIndices )
		{
			MRPT_UNUSED_PARAM(allData);
			MRPT_UNUSED_PARAM(useIndices);
			return false;
		}
	} // end namespace
} // end namespace


/*---------------------------------------------------------------
				ransac_detect_3D_planes
 ---------------------------------------------------------------*/
template <typename NUMTYPE>
void mrpt::math::ransac_detect_3D_planes(
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &z,
	vector<pair<size_t,TPlane> >   &out_detected_planes,
	const double           threshold,
	const size_t           min_inliers_for_valid_plane
	)
{
	MRPT_START

	ASSERT_(x.size()==y.size() && x.size()==z.size())

	out_detected_planes.clear();

	if (x.empty())
		return;

	// The running lists of remaining points after each plane, as a matrix:
	CMatrixTemplateNumeric<NUMTYPE> remainingPoints( 3, x.size() );
	remainingPoints.insertRow(0,x);
	remainingPoints.insertRow(1,y);
	remainingPoints.insertRow(2,z);


	// ---------------------------------------------
	// For each plane:
	// ---------------------------------------------
	for (;;)
	{
		mrpt::vector_size_t				this_best_inliers;
		CMatrixTemplateNumeric<NUMTYPE> this_best_model;

		math::RANSAC_Template<NUMTYPE> ransac;
		ransac.setVerbosityLevel(mrpt::utils::LVL_INFO);
		ransac.execute(
			remainingPoints,
			ransac3Dplane_fit,
			ransac3Dplane_distance,
			ransac3Dplane_degenerate,
			threshold,
			3,  // Minimum set of points
			this_best_inliers,
			this_best_model,
			0.999  // Prob. of good result
			);

		// Is this plane good enough?
		if (this_best_inliers.size()>=min_inliers_for_valid_plane)
		{
			// Add this plane to the output list:
			out_detected_planes.push_back(
				std::make_pair(
					this_best_inliers.size(),
					TPlane( double(this_best_model(0,0)), double(this_best_model(0,1)), double(this_best_model(0,2)), double(this_best_model(0,3)) )
					) );

			out_detected_planes.rbegin()->second.unitarize();

			// Discard the selected points so they are not used again for finding subsequent planes:
			remainingPoints.removeColumns(this_best_inliers);
		}
		else
		{
			break; // Do not search for more planes.
		}
	}

	MRPT_END
}



// Template explicit instantiations:
#define EXPLICIT_INST_ransac_detect_3D_planes(_TYPE_) \
	template void BASE_IMPEXP mrpt::math::ransac_detect_3D_planes<_TYPE_>( \
	const Eigen::Matrix<_TYPE_,Eigen::Dynamic,1>  &x, \
	const Eigen::Matrix<_TYPE_,Eigen::Dynamic,1>  &y, \
	const Eigen::Matrix<_TYPE_,Eigen::Dynamic,1>  &z, \
	vector<pair<size_t,TPlane> >   &out_detected_planes, \
	const double           threshold, \
	const size_t           min_inliers_for_valid_plane);

EXPLICIT_INST_ransac_detect_3D_planes(float)
EXPLICIT_INST_ransac_detect_3D_planes(double)
#ifdef HAVE_LONG_DOUBLE
EXPLICIT_INST_ransac_detect_3D_planes(long double)
#endif



/*---------------------------------------------------------------
		Aux. functions needed by ransac_detect_2D_lines
 ---------------------------------------------------------------*/
namespace mrpt
{
	namespace math
	{
		template <typename T>
		void  ransac2Dline_fit(
			const CMatrixTemplateNumeric<T> &allData,
			const vector_size_t  &useIndices,
			vector< CMatrixTemplateNumeric<T> > &fitModels )
		{
			ASSERT_(useIndices.size()==2);

			TPoint2D  p1( allData(0,useIndices[0]),allData(1,useIndices[0]) );
			TPoint2D  p2( allData(0,useIndices[1]),allData(1,useIndices[1]) );

			try
			{
				TLine2D  line(p1,p2);
				fitModels.resize(1);
				CMatrixTemplateNumeric<T> &M = fitModels[0];

				M.setSize(1,3);
				for (size_t i=0;i<3;i++)
					M(0,i)=line.coefs[i];
			}
			catch(exception &)
			{
				fitModels.clear();
				return;
			}
		}


		template <typename T>
		void ransac2Dline_distance(
			const CMatrixTemplateNumeric<T> &allData,
			const vector< CMatrixTemplateNumeric<T> > & testModels,
			const T distanceThreshold,
			unsigned int & out_bestModelIndex,
			vector_size_t & out_inlierIndices )
		{
			out_inlierIndices.clear();
			out_bestModelIndex = 0;

			if (testModels.empty()) return; // No model, no inliers.

			ASSERTMSG_( testModels.size()==1, format("Expected testModels.size()=1, but it's = %u",static_cast<unsigned int>(testModels.size()) ) )
			const CMatrixTemplateNumeric<T> &M = testModels[0];

			ASSERT_( size(M,1)==1 && size(M,2)==3 )

			TLine2D  line;
			line.coefs[0] = M(0,0);
			line.coefs[1] = M(0,1);
			line.coefs[2] = M(0,2);

			const size_t N = size(allData,2);
			out_inlierIndices.reserve(100);
			for (size_t i=0;i<N;i++)
			{
				const double d = line.distance( TPoint2D( allData.get_unsafe(0,i),allData.get_unsafe(1,i) ) );
				if (d<distanceThreshold)
					out_inlierIndices.push_back(i);
			}
		}

		/** Return "true" if the selected points are a degenerate (invalid) case.
		  */
		template <typename T>
		bool ransac2Dline_degenerate(
			const CMatrixTemplateNumeric<T> &allData,
			const mrpt::vector_size_t &useIndices )
		{
			MRPT_UNUSED_PARAM(allData);
			MRPT_UNUSED_PARAM(useIndices);
			return false;
		}
	} // end namespace
} // end namespace

/*---------------------------------------------------------------
				ransac_detect_2D_lines
 ---------------------------------------------------------------*/
template <typename NUMTYPE>
void mrpt::math::ransac_detect_2D_lines(
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &x,
	const Eigen::Matrix<NUMTYPE,Eigen::Dynamic,1>  &y,
	std::vector<std::pair<size_t,TLine2D> >   &out_detected_lines,
	const double           threshold,
	const size_t           min_inliers_for_valid_line
	)
{
	MRPT_START

	ASSERT_(x.size()==y.size())

	out_detected_lines.clear();

	if (x.empty())
		return;

	// The running lists of remaining points after each plane, as a matrix:
	CMatrixTemplateNumeric<NUMTYPE> remainingPoints( 2, x.size() );
	remainingPoints.insertRow(0,x);
	remainingPoints.insertRow(1,y);

	// ---------------------------------------------
	// For each line:
	// ---------------------------------------------
	while (size(remainingPoints,2)>=2)
	{
		mrpt::vector_size_t				this_best_inliers;
		CMatrixTemplateNumeric<NUMTYPE> this_best_model;

		math::RANSAC_Template<NUMTYPE> ransac;
		ransac.setVerbosityLevel(mrpt::utils::LVL_INFO);
		ransac.execute(
			remainingPoints,
			ransac2Dline_fit,
			ransac2Dline_distance,
			ransac2Dline_degenerate,
			threshold,
			2,  // Minimum set of points
			this_best_inliers,
			this_best_model,
			0.99999  // Prob. of good result
			);

		// Is this plane good enough?
		if (this_best_inliers.size()>=min_inliers_for_valid_line)
		{
			// Add this plane to the output list:
			out_detected_lines.push_back(
				std::make_pair(
					this_best_inliers.size(),
					TLine2D(double(this_best_model(0,0)), double(this_best_model(0,1)), double(this_best_model(0,2)) )
					) );

			out_detected_lines.rbegin()->second.unitarize();

			// Discard the selected points so they are not used again for finding subsequent planes:
			remainingPoints.removeColumns(this_best_inliers);
		}
		else
		{
			break; // Do not search for more planes.
		}
	}

	MRPT_END
}

// Template explicit instantiations:
#define EXPLICIT_INSTANT_ransac_detect_2D_lines(_TYPE_) \
	template void BASE_IMPEXP mrpt::math::ransac_detect_2D_lines<_TYPE_>( \
		const Eigen::Matrix<_TYPE_,Eigen::Dynamic,1>  &x, \
		const Eigen::Matrix<_TYPE_,Eigen::Dynamic,1>  &y, \
		std::vector<std::pair<size_t,TLine2D> >   &out_detected_lines, \
		const double           threshold, \
		const size_t           min_inliers_for_valid_line );  \

EXPLICIT_INSTANT_ransac_detect_2D_lines(float)
EXPLICIT_INSTANT_ransac_detect_2D_lines(double)
#ifdef HAVE_LONG_DOUBLE
	EXPLICIT_INSTANT_ransac_detect_2D_lines(long double)
#endif


