/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <cstddef>  // size_t

namespace mrpt::poses
{
// Values:
template <class DERIVEDCLASS, std::size_t DIM>
class CPoseOrPoint;
class CPoint2D;
class CPoint3D;
class CPose2D;
class CPose3D;
class CPose3DQuat;

// PDFs:
class CPointPDF;
struct CPointPDFPtr;
class CPosePDF;
struct CPosePDFPtr;
class CPose3DPDF;
struct CPose3DPDFPtr;
class CPose3DQuatPDF;
struct CPose3DQuatPDFPtr;
class CPosePDFParticles;
struct CPosePDFParticlesPtr;
class CPosePDFGaussian;
struct CPosePDFGaussianPtr;
class CPosePDFGaussianInf;
struct CPosePDFGaussianInfPtr;
class CPosePDFSOG;
struct CPosePDFSOGPtr;
class CPose3DPDF;
struct CPose3DPDFPtr;
class CPose3DPDFGaussian;
struct CPose3DPDFGaussianPtr;
class CPose3DPDFGaussianInf;
struct CPose3DPDFGaussianInfPtr;
class CPose3DQuatPDFGaussianInf;
struct CPose3DQuatPDFGaussianInfPtr;
}  // namespace mrpt::poses
