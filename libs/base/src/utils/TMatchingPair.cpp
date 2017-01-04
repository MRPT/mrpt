/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/system/os.h>
#include <mrpt/math/ops_containers.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
						dumpToFile
  ---------------------------------------------------------------*/
void  TMatchingPairList::dumpToFile(const std::string &fileName) const
{
	CFileOutputStream  f(fileName);
	ASSERT_(f.fileOpenCorrectly())

	for (const_iterator it=begin();it!=end();++it)
	{
		f.printf("%u %u %f %f %f %f %f %f %f\n",
				it->this_idx,
				it->other_idx,
				it->this_x,
				it->this_y,
				it->this_z,
				it->other_x,
				it->other_y,
				it->other_z,
				it->errorSquareAfterTransformation);
	}
}

/*---------------------------------------------------------------
						saveAsMATLABScript
  ---------------------------------------------------------------*/
void TMatchingPairList::saveAsMATLABScript( const std::string &filName ) const
{
	FILE	*f = os::fopen(filName.c_str(),"wt");

	fprintf(f,"%% ----------------------------------------------------\n");
	fprintf(f,"%%  File generated automatically by the MRPT method:\n");
	fprintf(f,"%%   saveAsMATLABScript  \n");
	fprintf(f,"%%  Before calling this script, define the color of lines, eg:\n");
	fprintf(f,"%%     colorLines=[1 1 1]");
	fprintf(f,"%%               J.L. Blanco (C) 2005-2012 \n");
	fprintf(f,"%% ----------------------------------------------------\n\n");

	fprintf(f,"axis equal; hold on;\n");
	for (const_iterator it=begin();it!=end();++it)
	{
		fprintf(f,"line([%f %f],[%f %f],'Color',colorLines);\n",
				it->this_x,
				it->other_x,
				it->this_y,
				it->other_y );
		fprintf(f,"set(plot([%f %f],[%f %f],'.'),'Color',colorLines,'MarkerSize',15);\n",
				it->this_x,
				it->other_x,
				it->this_y,
				it->other_y );
	}
	os::fclose(f);
}

/*---------------------------------------------------------------
						indexOtherMapHasCorrespondence
  ---------------------------------------------------------------*/
bool  TMatchingPairList::indexOtherMapHasCorrespondence(size_t idx) const
{
	for (const_iterator it=begin();it!=end();++it) {
		if (it->other_idx == idx) 
			return true;
	}
	return false;
}

bool mrpt::utils::operator < (const TMatchingPair& a, const TMatchingPair& b)
{
	if (a.this_idx==b.this_idx)
			return (a.this_idx<b.this_idx);
	else	return (a.other_idx<b.other_idx);
}


bool mrpt::utils::operator == (const TMatchingPair& a,const TMatchingPair& b)
{
	return (a.this_idx==b.this_idx) && (a.other_idx==b.other_idx);
}

bool mrpt::utils::operator == (const TMatchingPairList& a,const TMatchingPairList& b)
{
	if (a.size()!=b.size())
		return false;
	for (TMatchingPairList::const_iterator it1=a.begin(),it2=b.begin();it1!=a.end();++it1,++it2)
		if (!  ( (*it1)==(*it2)))
			return false;
	return true;
}


/*---------------------------------------------------------------
						overallSquareError
  ---------------------------------------------------------------*/
float TMatchingPairList::overallSquareError( const CPose2D &q ) const
{
	vector<float> errs( size() );
	squareErrorVector(q,errs);
	return math::sum( errs );
}

/*---------------------------------------------------------------
						overallSquareErrorAndPoints
  ---------------------------------------------------------------*/
float TMatchingPairList::overallSquareErrorAndPoints(
	const CPose2D &q,
	vector<float> &xs,
	vector<float> &ys ) const
{
	vector<float> errs( size() );
	squareErrorVector(q,errs,xs,ys);
	return math::sum( errs );
}

/*---------------------------------------------------------------
					TMatchingPairList::contains
  ---------------------------------------------------------------*/
bool TMatchingPairList::contains (const TMatchingPair &p) const
{
	for (const_iterator corresp=begin();corresp!=end();++corresp)
		if ( *corresp == p )
			return true;
	return false;
}

/*---------------------------------------------------------------
						squareErrorVector
  ---------------------------------------------------------------*/
void  TMatchingPairList::squareErrorVector(const CPose2D &q, vector<float> &out_sqErrs ) const
{
	out_sqErrs.resize( size() );
	// *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]

	const float ccos = cos(q.phi());
	const float csin = sin(q.phi());
	const float qx   = q.x();
	const float qy   = q.y();

	const_iterator 			corresp;
	vector<float>::iterator	e_i;
	for (corresp=begin(), e_i = out_sqErrs.begin();corresp!=end();++corresp, ++e_i)
	{
		float xx = qx + ccos * corresp->other_x - csin * corresp->other_y;
		float yy = qy + csin * corresp->other_x + ccos * corresp->other_y;
		*e_i = square( corresp->this_x - xx ) + square( corresp->this_y - yy );
	}
}

/*---------------------------------------------------------------
						squareErrorVector
  ---------------------------------------------------------------*/
void  TMatchingPairList::squareErrorVector(
	const CPose2D &q,
	vector<float> &out_sqErrs,
	vector<float> &xs,
	vector<float> &ys ) const
{
	out_sqErrs.resize( size() );
	xs.resize( size() );
	ys.resize( size() );

	// *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]

	const float ccos = cos(q.phi());
	const float csin = sin(q.phi());
	const float qx   = q.x();
	const float qy   = q.y();

	const_iterator 			corresp;
	vector<float>::iterator	e_i, xx, yy;
	for (corresp=begin(), e_i = out_sqErrs.begin(), xx = xs.begin(), yy = ys.begin();corresp!=end();++corresp, ++e_i, ++xx,++yy)
	{
		*xx = qx + ccos * corresp->other_x - csin * corresp->other_y;
		*yy = qy + csin * corresp->other_x + ccos * corresp->other_y;
		*e_i = square( corresp->this_x - *xx ) + square( corresp->this_y - *yy );
	}
}


