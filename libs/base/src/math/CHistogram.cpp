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


#include <mrpt/math/CHistogram.h>
#include <mrpt/math/ops_vectors.h>
#include <mrpt/math/utils.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/metaprogramming.h>

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;

/*---------------------------------------------------------------
					Constructor
 ---------------------------------------------------------------*/
CHistogram::CHistogram(const double min, const double max, const size_t nBins) :
	m_min(min), m_max(max), m_bins(nBins,0), m_count(0)
{
	ASSERT_(nBins>0)
	ASSERT_(max>min)
	m_binSizeInv = (nBins-1) / (m_max-m_min);
}

/*---------------------------------------------------------------
						clear
 ---------------------------------------------------------------*/
void CHistogram::clear()
{
	m_bins.assign(m_bins.size(), 0 );
	m_count = 0;
}

/*---------------------------------------------------------------
							add
 ---------------------------------------------------------------*/
void CHistogram::add(const double x)
{
	if (x<m_min || x>m_max) return;

	size_t ind = static_cast<size_t>( m_binSizeInv * (x - m_min) );

	m_bins[ind]++;
	m_count++;
}

/*---------------------------------------------------------------
					getBinCount
 ---------------------------------------------------------------*/
int CHistogram::getBinCount(const size_t index) const
{
	if (index>=m_bins.size()) THROW_EXCEPTION("Index out of bounds")

	return m_bins[index];
}

/*---------------------------------------------------------------
					getBinRatio
 ---------------------------------------------------------------*/
double CHistogram::getBinRatio(const size_t index) const
{
	if (index>=m_bins.size()) THROW_EXCEPTION("Index out of bounds")

	if (m_count)	return m_bins[index]/double(m_count);
	else		return 0;
}

/*---------------------------------------------------------------
					getHistogram
 ---------------------------------------------------------------*/
void CHistogram::getHistogram( vector_double &x, vector_double &hits ) const
{
	linspace(m_min,m_max,m_bins.size(), x);
	metaprogramming::copy_container_typecasting(m_bins,hits);
}


/*---------------------------------------------------------------
					getHistogramNormalized
 ---------------------------------------------------------------*/
void CHistogram::getHistogramNormalized( vector_double &x, vector_double &hits ) const
{
	const size_t N = m_bins.size();
	linspace(m_min,m_max,N, x);

	hits.resize(N);
	const double K=m_binSizeInv/m_count;
	for (size_t i=0;i<N;i++)
		hits[i]=K*m_bins[i];
}

