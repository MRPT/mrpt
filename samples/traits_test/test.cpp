/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPoint2DPDF.h>
#include <mrpt/poses/CPoint2DPDFGaussian.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPointPDF.h>
#include <mrpt/poses/CPointPDFGaussian.h>
#include <mrpt/poses/CPointPDFParticles.h>
#include <mrpt/poses/CPointPDFSOG.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose2DGridTemplate.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <type_traits>



/* ------------------------------------------------------------------------
					Test: Traits

		Tests to make sure that certain classes have to desired traits.
   ------------------------------------------------------------------------ */


template<typename T>
class TraitsTest{
  static_assert(std::is_move_constructible<T>(), "Can't move");
  static_assert(std::is_copy_constructible<T>(), "Can't copy");
  static_assert(std::is_move_assignable<T>(), "Can't move assign");
  static_assert(std::is_copy_assignable<T>(), "Can't copy assign");
};

template class TraitsTest<mrpt::poses::CPoint2D>;
template class TraitsTest<mrpt::poses::CPoint2DPDF>;
template class TraitsTest<mrpt::poses::CPoint2DPDFGaussian>;
template class TraitsTest<mrpt::poses::CPoint3D>;
template class TraitsTest<mrpt::poses::CPointPDF>;
template class TraitsTest<mrpt::poses::CPointPDFGaussian>;
template class TraitsTest<mrpt::poses::CPointPDFParticles>;
template class TraitsTest<mrpt::poses::CPointPDFSOG>;
template class TraitsTest<mrpt::poses::CPose2D>;
template class TraitsTest<mrpt::poses::CPose2DGridTemplate<double> >;
template class TraitsTest<mrpt::poses::CPose3D>;
template class TraitsTest<mrpt::poses::CPose3DInterpolator>;
template class TraitsTest<mrpt::poses::CPose3DPDF>;

// ------------------------------------------------------
//						MAIN
// ------------------------------------------------------
//Dummy main since all tests are static and run at compile time. 
int main(int argc, char **argv)
{
}
