/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2021, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "slam-precomp.h"  // Precompiled headers

#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/slam/CMetricMapsAlignmentAlgorithm.h>

using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::math;
using namespace mrpt::poses;

CPosePDF::Ptr CMetricMapsAlignmentAlgorithm::Align(
	const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
	const CPose2D& grossEst,
	mrpt::optional_ref<TMetricMapAlignmentResult> outInfo)
{
	CPosePDFGaussian posePDF(grossEst, CMatrixDouble33());
	return AlignPDF(m1, m2, posePDF, outInfo);
}

CPose3DPDF::Ptr CMetricMapsAlignmentAlgorithm::Align3D(
	const mrpt::maps::CMetricMap* m1, const mrpt::maps::CMetricMap* m2,
	const CPose3D& grossEst,
	mrpt::optional_ref<TMetricMapAlignmentResult> outInfo)
{
	CPose3DPDFGaussian posePDF;
	posePDF.mean = grossEst;
	return Align3DPDF(m1, m2, posePDF, outInfo);
}
