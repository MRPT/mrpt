/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "obs-precomp.h"   // Precompiled headers

#include <mrpt/obs/CObservation2DRangeScanWithUncertainty.h>

using namespace mrpt::obs;

double CObservation2DRangeScanWithUncertainty::evaluateScanLikelihood(const CObservation2DRangeScan& otherScan, double outliers_prob) const
{
	ASSERT_EQUAL_( otherScan.scan.size(),otherScan.validRange.size() );
	ASSERT_EQUAL_( otherScan.scan.size(), this->rangesMean.size() );
	ASSERT_EQUAL_( otherScan.scan.size(), this->rangesCovar.rows() );
	ASSERT_EQUAL_( otherScan.scan.size(), this->rangesCovar.cols() );
	ASSERT_(outliers_prob>=0.0 && outliers_prob<=1.0)
	ASSERT_(otherScan.maxRange>0.0)

	const size_t N = rangesMean.size();
	double lik_prod = 1.0;
	double lik_avrg = .0;
	const double outlier_const_term = 1.0/otherScan.maxRange;
	for (size_t i=0;i<N;i++)
	{
		const double iLikGauss = exp( -0.5 * mrpt::utils::square( otherScan.scan[i] - rangesMean[i] )/ rangesCovar(i,i) );

		double iLik;
		if (otherScan.scan[i] > rangesMean[i]) {
			iLik = iLikGauss; 
		} else {
			iLik = std::max(outlier_const_term, iLikGauss);
		}
		lik_prod *= iLik;
		lik_avrg += iLik;
	}
	lik_avrg/=N;

	return lik_avrg;
}

