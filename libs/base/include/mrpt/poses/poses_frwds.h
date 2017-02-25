/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#pragma once

namespace mrpt
{
	namespace poses
	{
		// Values:
		template <class DERIVEDCLASS> class CPoseOrPoint;
		class CPoint2D;
		class CPoint3D;
		class CPose2D;
		class CPose3D;
		class CPose3DQuat;
		class CPose3DRotVec;

		// PDFs:
		class CPointPDF; struct CPointPDFPtr;
		class CPosePDF;  struct CPosePDFPtr;
		class CPose3DPDF; struct CPose3DPDFPtr;
		class CPose3DQuatPDF; struct CPose3DQuatPDFPtr;
		class CPosePDFParticles; struct CPosePDFParticlesPtr;
		class CPosePDFGaussian; struct CPosePDFGaussianPtr;
		class CPosePDFGaussianInf; struct CPosePDFGaussianInfPtr;
		class CPosePDFSOG; struct CPosePDFSOGPtr;
		class CPose3DPDF; struct CPose3DPDFPtr;
		class CPose3DPDFGaussian; struct CPose3DPDFGaussianPtr;
		class CPose3DPDFGaussianInf; struct CPose3DPDFGaussianInfPtr;

	}
} 
