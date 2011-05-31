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

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/utils/TMatchingPair.h>
#include <mrpt/utils/CFileOutputStream.h>
#include <mrpt/utils/utils_defs.h>
#include <mrpt/math/utils.h>
#include <mrpt/poses/CPose2D.h>

using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::system;
using namespace std;

/*---------------------------------------------------------------
						dumpToFile
  ---------------------------------------------------------------*/
void  TMatchingPairList::dumpToFile(const std::string &fileName)
{
	CFileOutputStream  f(fileName);
	ASSERT_(f.fileOpenCorrectly())

	for (iterator it=begin();it!=end();it++)
	{
		f.printf("%u %u %f %f %f %f %f %f\n",
				it->this_idx,
				it->other_idx,
				it->this_x,
				it->this_y,
				it->this_z,
				it->other_x,
				it->other_y,
				it->other_z);
	}
}

/*---------------------------------------------------------------
						saveAsMATLABScript
  ---------------------------------------------------------------*/
void TMatchingPairList::saveAsMATLABScript( const std::string &filName )
{
	FILE	*f = os::fopen(filName.c_str(),"wt");

	fprintf(f,"%% ----------------------------------------------------\n");
	fprintf(f,"%%  File generated automatically by the MRPT method:\n");
	fprintf(f,"%%   saveAsMATLABScript  \n");
	fprintf(f,"%%  Before calling this script, define the color of lines, eg:\n");
	fprintf(f,"%%     colorLines=[1 1 1]");
	fprintf(f,"%%               J.L. Blanco (C) 2005-2011 \n");
	fprintf(f,"%% ----------------------------------------------------\n\n");

	fprintf(f,"axis equal; hold on;\n");
	iterator	it;
	for (it=begin();it!=end();it++)
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
bool  TMatchingPairList::indexOtherMapHasCorrespondence(unsigned int idx)
{
	iterator	it;
	bool		has = false;

	for (it=begin();it!=end() && !has;it++)
	{
		has = it->other_idx == idx;
	}

	return has;
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
	for (TMatchingPairList::const_iterator it1=a.begin(),it2=b.begin();it1!=a.end();it1++,it2++)
		if (!  ( (*it1)==(*it2)))
			return false;
	return true;
}


/*---------------------------------------------------------------
						overallSquareError
  ---------------------------------------------------------------*/
float TMatchingPairList::overallSquareError( const CPose2D &q ) const
{
	vector_float errs( size() );
	squareErrorVector(q,errs);
	return math::sum( errs );
}

/*---------------------------------------------------------------
						overallSquareErrorAndPoints
  ---------------------------------------------------------------*/
float TMatchingPairList::overallSquareErrorAndPoints(
	const CPose2D &q,
	vector_float &xs,
	vector_float &ys ) const
{
	vector_float errs( size() );
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
void  TMatchingPairList::squareErrorVector(const CPose2D &q, vector_float &out_sqErrs ) const
{
	out_sqErrs.resize( size() );
	// *    \f[ e_i = | x_{this} -  q \oplus x_{other}  |^2 \f]

	const float ccos = cos(q.phi());
	const float csin = sin(q.phi());
	const float qx   = q.x();
	const float qy   = q.y();
	float  xx, yy;		// Transformed points

	const_iterator 			corresp;
	vector_float::iterator	e_i;
	for (corresp=begin(), e_i = out_sqErrs.begin();corresp!=end();corresp++, e_i++)
	{
		xx = qx + ccos * corresp->other_x - csin * corresp->other_y;
		yy = qy + csin * corresp->other_x + ccos * corresp->other_y;
		*e_i = square( corresp->this_x - xx ) + square( corresp->this_y - yy );
	}
}

/*---------------------------------------------------------------
						squareErrorVector
  ---------------------------------------------------------------*/
void  TMatchingPairList::squareErrorVector(
	const CPose2D &q,
	vector_float &out_sqErrs,
	vector_float &xs,
	vector_float &ys ) const
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
	vector_float::iterator	e_i, xx, yy;
	for (corresp=begin(), e_i = out_sqErrs.begin(), xx = xs.begin(), yy = ys.begin();corresp!=end();corresp++, e_i++, xx++,yy++)
	{
		*xx = qx + ccos * corresp->other_x - csin * corresp->other_y;
		*yy = qy + csin * corresp->other_x + ccos * corresp->other_y;
		*e_i = square( corresp->this_x - *xx ) + square( corresp->this_y - *yy );
	}
}


