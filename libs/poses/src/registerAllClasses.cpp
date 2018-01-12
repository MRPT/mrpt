/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "poses-precomp.h"

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/poses/CPoses2DSequence.h>
#include <mrpt/poses/CPoses3DSequence.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPosePDFParticles.h>
#include <mrpt/poses/CPosePDFGrid.h>
#include <mrpt/poses/CPosePDFSOG.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFParticles.h>
#include <mrpt/poses/CPose3DPDFSOG.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPose3DQuatPDFGaussianInf.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/poses/CPose3DInterpolator.h>

#include <mrpt/core/initializer.h>

using namespace mrpt::poses;

MRPT_INITIALIZER(registerAllClasses_mrpt_poses)
{
#if !defined(DISABLE_MRPT_AUTO_CLASS_REGISTRATION)
	registerClass(CLASS_ID(CPoint2D));
	registerClass(CLASS_ID(CPoint3D));
	registerClass(CLASS_ID(CPose2D));
	registerClass(CLASS_ID(CPose3D));
	registerClass(CLASS_ID(CPose3DQuat));
	registerClass(CLASS_ID(CPoses2DSequence));
	registerClass(CLASS_ID(CPoses3DSequence));

	registerClass(CLASS_ID(CPosePDF));
	registerClass(CLASS_ID(CPosePDFGaussian));
	registerClass(CLASS_ID(CPosePDFGaussianInf));
	registerClass(CLASS_ID(CPosePDFParticles));
	registerClass(CLASS_ID(CPosePDFGrid));
	registerClass(CLASS_ID(CPosePDFSOG));

	registerClass(CLASS_ID(CPointPDF));
	registerClass(CLASS_ID(CPointPDFGaussian));
	registerClass(CLASS_ID(CPointPDFParticles));
	registerClass(CLASS_ID(CPointPDFSOG));

	registerClass(CLASS_ID(CPosePDF));
	registerClass(CLASS_ID(CPose3DPDF));
	registerClass(CLASS_ID(CPose3DQuatPDF));
	registerClass(CLASS_ID(CPose3DPDFGaussian));
	registerClass(CLASS_ID(CPose3DPDFGaussianInf));
	registerClass(CLASS_ID(CPose3DPDFParticles));
	registerClass(CLASS_ID(CPose3DPDFSOG));

	registerClass(CLASS_ID(CPose3DQuatPDF));
	registerClass(CLASS_ID(CPose3DQuatPDFGaussian));
	registerClass(CLASS_ID(CPose3DQuatPDFGaussianInf));

	registerClass(CLASS_ID(CPose2DInterpolator));
	registerClass(CLASS_ID(CPose3DInterpolator));
#endif
}
