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
		class CPointPDF; 
		class CPosePDF;  
		class CPose3DPDF; 
		class CPose3DQuatPDF; 
		class CPosePDFParticles; 
		class CPosePDFGaussian; 
		class CPosePDFGaussianInf; 
		class CPosePDFSOG; 
		class CPose3DPDF; 
		class CPose3DPDFGaussian; 
		class CPose3DPDFGaussianInf; 

	}
} 
