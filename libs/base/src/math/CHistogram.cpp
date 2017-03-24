/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers
#include <mrpt/utils/types_math.h>

#include <mrpt/math/CHistogram.h>
#include <mrpt/math/utils.h>

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
	m_binSizeInv = nBins / (m_max-m_min);
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
size_t CHistogram::getBinCount(const size_t index) const
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
void CHistogram::getHistogram( std::vector<double> &x, std::vector<double> &hits ) const
{
	linspace(m_min,m_max,m_bins.size(), x);
	const size_t N= m_bins.size();
	hits.resize(N);
	for (size_t i=0;i<N;i++) hits[i] = static_cast<double>(m_bins[i]); // metaprogramming::copy_container_typecasting(m_bins,);
}


/*---------------------------------------------------------------
					getHistogramNormalized
 ---------------------------------------------------------------*/
void CHistogram::getHistogramNormalized( std::vector<double> &x, std::vector<double> &hits ) const
{
	const size_t N = m_bins.size();
	linspace(m_min,m_max,N, x);

	hits.resize(N);
	const double K=m_binSizeInv/m_count;
	for (size_t i=0;i<N;i++)
		hits[i]=K*m_bins[i];
}

