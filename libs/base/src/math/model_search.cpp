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

#include <mrpt/math/utils.h>
#include <mrpt/math/model_search.h>

using namespace mrpt;
using namespace mrpt::math;

//----------------------------------------------------------------------
//! Select random (unique) indices from the 0..p_size sequence
void ModelSearch::pickRandomIndex( size_t p_size, size_t p_pick, vector_size_t& p_ind )
{
	ASSERT_( p_size >= p_pick );

	vector_size_t a( p_size );
	for( size_t i = 0; i < p_size; i++ )
		a[i] = i;

	std::random_shuffle( a.begin(), a.end() );
	p_ind.resize( p_pick );
	for( size_t i = 0 ; i < p_pick; i++ )
		p_ind[i] = a[i];
}

//----------------------------------------------------------------------
//! Select random (unique) indices from the set.
//! The set is destroyed during pick
void ModelSearch::pickRandomIndex( std::set<size_t> p_set, size_t p_pick, vector_size_t& p_ind )
{
	p_ind.resize( p_pick );
	vector_size_t inds( p_set.begin(), p_set.end() );

	std::random_shuffle( inds.begin(), inds.end() );
	p_ind.resize( p_pick );
	for( size_t i = 0 ; i < p_pick; i++ )
		p_ind[i] = inds[i];
}

