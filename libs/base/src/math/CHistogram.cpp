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

