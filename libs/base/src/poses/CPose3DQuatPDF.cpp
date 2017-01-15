/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers

#include <mrpt/utils/CStream.h>
#include <mrpt/poses/CPose3DQuatPDF.h>
#include <mrpt/poses/CPose3DQuatPDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>

using namespace mrpt::poses;
using namespace mrpt::math;
using namespace std;

IMPLEMENTS_VIRTUAL_SERIALIZABLE( CPose3DQuatPDF, CSerializable, mrpt::poses )

/*---------------------------------------------------------------
					copyFrom2D
  ---------------------------------------------------------------*/
CPose3DQuatPDF* CPose3DQuatPDF::createFrom2D(const CPosePDF &o)
{
	MRPT_START

	CPose3DPDFGaussian q;
	q.copyFrom(o);

	return new CPose3DQuatPDFGaussian(q);

	MRPT_END
}


/*---------------------------------------------------------------
					jacobiansPoseComposition
 ---------------------------------------------------------------*/
void CPose3DQuatPDF::jacobiansPoseComposition(
	const CPose3DQuat &x,
	const CPose3DQuat &u,
	CMatrixDouble77	  &df_dx,
	CMatrixDouble77	  &df_du,
	CPose3DQuat       *out_x_oplus_u
	)
{
	// For the derivation of the formulas, see the tech. report cited in the header file.
	const double   qr = x.quat().r();
	const double   qx = x.quat().x(); const double qx2 = square(qx);
	const double   qy = x.quat().y(); const double qy2 = square(qy);
	const double   qz = x.quat().z(); const double qz2 = square(qz);

	const double   ax  = u.x();
	const double   ay  = u.y();
	const double   az  = u.z();
	const double   q2r = u.quat().r();
	const double   q2x = u.quat().x();
	const double   q2y = u.quat().y();
	const double   q2z = u.quat().z();

	CPose3DQuat  x_plus_u = x + u;  // needed for the normalization Jacobian:
	CMatrixDouble44  norm_jacob(UNINITIALIZED_MATRIX);
	x_plus_u.quat().normalizationJacobian(norm_jacob);

	CMatrixDouble44  norm_jacob_x(UNINITIALIZED_MATRIX);
	x.quat().normalizationJacobian(norm_jacob_x);

	// df_dx ===================================================
	df_dx.zeros();

	// first part 3x7:  df_{qr} / dp
	df_dx.set_unsafe(0,0, 1);
	df_dx.set_unsafe(1,1, 1);
	df_dx.set_unsafe(2,2, 1);

	MRPT_ALIGN16 const double vals2[3*4] = {
		2*(-qz*ay +qy*az  ),
		2*(qy*ay + qz*az  ),
		2*(-2*qy*ax + qx*ay +qr*az  ),
		2*(-2*qz*ax - qr*ay +qx*az  ),

		2*(qz*ax - qx*az   ),
		2*(qy*ax - 2*qx*ay -qr*az  ),
		2*(qx*ax +qz*az   ),
		2*(qr*ax - 2*qz*ay +qy*az ),

		2*(-qy*ax + qx*ay  ),
		2*( qz*ax + qr*ay - 2*qx*az  ),
		2*(-qr*ax + qz*ay - 2*qy*az  ),
		2*( qx*ax + qy*ay )
		};

	// df_dx(0:3,3:7) = vals2 * NORM_JACOB
	df_dx.block(0,3, 3,4).noalias() = (CMatrixFixedNumeric<double,3,4>(vals2) * norm_jacob_x).eval();

	// second part:
	{
		MRPT_ALIGN16 const double aux44_data[4*4] = {
			q2r,-q2x,-q2y,-q2z,
			q2x, q2r, q2z,-q2y,
			q2y,-q2z, q2r, q2x,
			q2z, q2y,-q2x, q2r };

		df_dx.block(3,3, 4,4).noalias() =  (norm_jacob * CMatrixFixedNumeric<double,4,4>(aux44_data)).eval();
	}

	// df_du ===================================================
	df_du.zeros();

	// first part 3x3:  df_{qr} / da
	df_du.set_unsafe(0,0,  1-2*(qy2+qz2) );
	df_du.set_unsafe(0,1,  2*(qx*qy - qr*qz ) );
	df_du.set_unsafe(0,2,  2*(qr*qy + qx*qz ) );

	df_du.set_unsafe(1,0,  2*(qr*qz + qx*qy  ) );
	df_du.set_unsafe(1,1,  1 - 2*( qx2+qz2) );
	df_du.set_unsafe(1,2,  2*(qy*qz - qr*qx ) );

	df_du.set_unsafe(2,0,  2*(qx*qz - qr*qy ) );
	df_du.set_unsafe(2,1,  2*(qr*qx + qy*qz ) );
	df_du.set_unsafe(2,2,  1-2*(qx2+qy2) );

	// Second part:
	{
		MRPT_ALIGN16 const double aux44_data[4*4] = {
			qr,-qx,-qy,-qz,
			qx, qr,-qz, qy,
			qy, qz, qr,-qx,
			qz,-qy, qx, qr };

		df_du.block(3,3, 4,4).noalias() = (norm_jacob * CMatrixFixedNumeric<double,4,4>(aux44_data)).eval();
	}

	if (out_x_oplus_u)
		*out_x_oplus_u = x_plus_u;
}

