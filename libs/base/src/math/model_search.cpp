/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2012, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2012, MAPIR group, University of Malaga                |
   | Copyright (c) 2012, University of Almeria                                 |
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

