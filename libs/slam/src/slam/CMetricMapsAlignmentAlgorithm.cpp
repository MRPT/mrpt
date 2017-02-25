/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "slam-precomp.h"   // Precompiled headers

#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;

/*---------------------------------------------------------------
					Align
  ---------------------------------------------------------------*/
CPosePDFPtr CMetricMapsAlignmentAlgorithm::Align(
    const mrpt::maps::CMetricMap		*m1,
    const mrpt::maps::CMetricMap		*m2,
    const CPose2D			&grossEst,
    float					*runningTime,
    void					*info )
{
	CPosePDFGaussian    posePDF(grossEst, CMatrixDouble33() );
	return AlignPDF(m1,m2,posePDF,runningTime,info);
}

/*---------------------------------------------------------------
					Align3D
  ---------------------------------------------------------------*/
CPose3DPDFPtr CMetricMapsAlignmentAlgorithm::Align3D(
    const mrpt::maps::CMetricMap		*m1,
    const mrpt::maps::CMetricMap		*m2,
    const CPose3D			&grossEst,
    float					*runningTime,
    void					*info )
{
	CPose3DPDFGaussian    posePDF;
	posePDF.mean = grossEst;
	return Align3DPDF(m1,m2,posePDF,runningTime,info);
}


