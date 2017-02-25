/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "base-precomp.h"  // Precompiled headers
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>
#include <mrpt/poses/CPose3DQuat.h>
#include <mrpt/math/CMatrix.h>
#include <mrpt/utils/CStream.h>
#include <iomanip>
#include <limits>

using namespace std;
using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;


IMPLEMENTS_SERIALIZABLE(CPose3DQuat, CSerializable, mrpt::poses)

/** Constructor from a CPose3D */
CPose3DQuat::CPose3DQuat(const CPose3D &p)
{
	x() = p.x();
	y() = p.y();
	z() = p.z();
	p.getAsQuaternion(m_quat);
}

/** Constructor from a 4x4 homogeneous transformation matrix.
  */
CPose3DQuat::CPose3DQuat(const CMatrixDouble44 &M) : m_quat(UNINITIALIZED_QUATERNION)
{
	m_coords[0] = M.get_unsafe(0,3);
	m_coords[1] = M.get_unsafe(1,3);
	m_coords[2] = M.get_unsafe(2,3);
	CPose3D p(M);
	p.getAsQuaternion(m_quat);
}

/** Returns the corresponding 4x4 homogeneous transformation matrix for the point(translation) or pose (translation+orientation).
  * \sa getInverseHomogeneousMatrix
  */
void  CPose3DQuat::getHomogeneousMatrix(CMatrixDouble44 & out_HM ) const
{
	m_quat.rotationMatrixNoResize(out_HM);
	out_HM.get_unsafe(0,3) = m_coords[0];
	out_HM.get_unsafe(1,3) = m_coords[1];
	out_HM.get_unsafe(2,3) = m_coords[2];
	out_HM.get_unsafe(3,0) = out_HM.get_unsafe(3,1) = out_HM.get_unsafe(3,2) = 0;
	out_HM.get_unsafe(3,3) = 1;
}

/** Returns a 1x7 vector with [x y z qr qx qy qz] */
void CPose3DQuat::getAsVector(CVectorDouble &v) const
{
	v.resize(7);
	v[0] = m_coords[0];
	v[1] = m_coords[1];
	v[2] = m_coords[2];
	v[3] = m_quat[0];
	v[4] = m_quat[1];
	v[5] = m_quat[2];
	v[6] = m_quat[3];
}

/**  Makes "this = A (+) B"; this method is slightly more efficient than "this= A + B;" since it avoids the temporary object.
  *  \note A or B can be "this" without problems.
  */
void CPose3DQuat::composeFrom(const CPose3DQuat& A, const CPose3DQuat& B )
{
	// The 3D point:
	double gx,gy,gz;
	A.m_quat.rotatePoint(B.m_coords[0],B.m_coords[1],B.m_coords[2], gx,gy,gz);
	this->m_coords[0] = A.m_coords[0] + gx;
	this->m_coords[1] = A.m_coords[1] + gy;
	this->m_coords[2] = A.m_coords[2] + gz;

	// The 3D rotation:
	this->m_quat.crossProduct(A.m_quat,B.m_quat);
}

/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
  *  \note A or B can be "this" without problems.
  * \sa composeFrom
  */
void CPose3DQuat::inverseComposeFrom(const CPose3DQuat& A, const CPose3DQuat& B )
{
	// The 3D point:
	const CQuaternionDouble B_conj(B.m_quat.r(),-B.m_quat.x(),-B.m_quat.y(),-B.m_quat.z());
	B_conj.rotatePoint(A.m_coords[0]-B.m_coords[0],A.m_coords[1]-B.m_coords[1],A.m_coords[2]-B.m_coords[2],  this->m_coords[0],this->m_coords[1],this->m_coords[2]);
	// The 3D rotation:
	this->m_quat.crossProduct(B_conj,A.m_quat);
}


/**  Computes the 3D point G such as \f$ G = this \oplus L \f$.
  * \sa inverseComposeFrom
  */
void CPose3DQuat::composePoint(const double lx,const double ly,const double lz,double &gx,double &gy,double &gz,
	mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double,3,7>  *out_jacobian_df_dpose ) const
{
	if (out_jacobian_df_dpoint || out_jacobian_df_dpose)
	{
		const double qx2 = square(m_quat.x());
		const double qy2 = square(m_quat.y());
		const double qz2 = square(m_quat.z());

		// Jacob: df/dpoint
		if (out_jacobian_df_dpoint)
		{
			// 3x3:  df_{qr} / da

			MRPT_ALIGN16 const double vals[3*3] = {
				1-2*(qy2+qz2) ,
				2*(m_quat.x()*m_quat.y() - m_quat.r()*m_quat.z() ) ,
				2*(m_quat.r()*m_quat.y() + m_quat.x()*m_quat.z() ) ,

				2*(m_quat.r()*m_quat.z() + m_quat.x()*m_quat.y()  ) ,
				1 - 2*( qx2+qz2) ,
				2*(m_quat.y()*m_quat.z() - m_quat.r()*m_quat.x() ) ,

				2*(m_quat.x()*m_quat.z() - m_quat.r()*m_quat.y() ) ,
				2*(m_quat.r()*m_quat.x() + m_quat.y()*m_quat.z() ) ,
				1-2*(qx2+qy2)
				};
			out_jacobian_df_dpoint->loadFromArray(vals);
		}

		// Jacob: df/dpose
		if (out_jacobian_df_dpose)
		{
			// 3x7:  df_{qr} / dp
			MRPT_ALIGN16 const double vals1[3*7] = {
				1,0,0, 0,0,0,0,
				0,1,0, 0,0,0,0,
				0,0,1, 0,0,0,0 };
			out_jacobian_df_dpose->loadFromArray(vals1);


			MRPT_ALIGN16 const double vals[3*4] = {
				2*(-m_quat.z()*ly +m_quat.y()*lz  ),
				2*(m_quat.y()*ly + m_quat.z()*lz  ),
				2*(-2*m_quat.y()*lx + m_quat.x()*ly +m_quat.r()*lz  ),
				2*(-2*m_quat.z()*lx - m_quat.r()*ly +m_quat.x()*lz  ),

				2*(m_quat.z()*lx - m_quat.x()*lz   ),
				2*(m_quat.y()*lx - 2*m_quat.x()*ly -m_quat.r()*lz  ),
				2*(m_quat.x()*lx +m_quat.z()*lz   ),
				2*(m_quat.r()*lx - 2*m_quat.z()*ly +m_quat.y()*lz ),

				2*(-m_quat.y()*lx + m_quat.x()*ly  ),
				2*( m_quat.z()*lx + m_quat.r()*ly - 2*m_quat.x()*lz  ),
				2*(-m_quat.r()*lx + m_quat.z()*ly - 2*m_quat.y()*lz  ),
				2*( m_quat.x()*lx + m_quat.y()*ly )
				};

			CMatrixDouble44  norm_jacob(UNINITIALIZED_MATRIX);
			this->quat().normalizationJacobian(norm_jacob);

			out_jacobian_df_dpose->insertMatrix(0,3, (CMatrixFixedNumeric<double,3,4>(vals)*norm_jacob).eval() );
		}
	}

	// function itself:
	m_quat.rotatePoint(lx,ly,lz, gx,gy,gz);
	gx+=m_coords[0];
	gy+=m_coords[1];
	gz+=m_coords[2];
}

/**  Computes the 3D point G such as \f$ L = G \ominus this \f$.
  * \sa composeFrom
  */
void CPose3DQuat::inverseComposePoint(const double gx,const double gy,const double gz,double &lx,double &ly,double &lz,
	mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double,3,7>  *out_jacobian_df_dpose) const
{
	if (out_jacobian_df_dpoint || out_jacobian_df_dpose)
	{
		const double qx2 = square(m_quat.x());
		const double qy2 = square(m_quat.y());
		const double qz2 = square(m_quat.z());

		// Jacob: df/dpoint
		if (out_jacobian_df_dpoint)
		{
			// 3x3:  df_{m_quat.r()} / da
			//		inv_df_da =
			//		[ - 2*qy^2 - 2*qz^2 + 1,     2*qx*qy - 2*qr*qz,     2*qr*qy + 2*qx*qz]
			//		[     2*qr*qz + 2*qx*qy, - 2*qx^2 - 2*qz^2 + 1,     2*qy*qz - 2*qr*qx]
			//		[     2*qx*qz - 2*qr*qy,     2*qr*qx + 2*qy*qz, - 2*qx^2 - 2*qy^2 + 1]
			//

			MRPT_ALIGN16 const double vals[3*3] = {
				1-2*(qy2+qz2),
				2*(m_quat.x()*m_quat.y() + m_quat.r()*m_quat.z() ) ,
				2*(-m_quat.r()*m_quat.y() + m_quat.x()*m_quat.z() ) ,

				2*(-m_quat.r()*m_quat.z() + m_quat.x()*m_quat.y()  ) ,
				1 - 2*( qx2+qz2) ,
				2*(m_quat.y()*m_quat.z() + m_quat.r()*m_quat.x() ) ,

				2*(m_quat.x()*m_quat.z() + m_quat.r()*m_quat.y() ) ,
				2*(-m_quat.r()*m_quat.x() + m_quat.y()*m_quat.z() ) ,
				1-2*(qx2+qy2)
				};
			out_jacobian_df_dpoint->loadFromArray(vals);
		}

		// Jacob: df/dpose
		if (out_jacobian_df_dpose)
		{
			// 3x7:  df_{m_quat.r()} / dp
			//		inv_df_dp =
			//[ 2*qy^2 + 2*qz^2 - 1, - 2*qr*qz - 2*qx*qy,   2*qr*qy - 2*qx*qz, 2*qz*(ay - y) - 2*qy*(az - z),                 2*qy*(ay - y) + 2*qz*(az - z), 2*qx*(ay - y) - 4*qy*(ax - x) - 2*qr*(az - z), 2*qr*(ay - y) - 4*qz*(ax - x) + 2*qx*(az - z)]
			//[   2*qr*qz - 2*qx*qy, 2*qx^2 + 2*qz^2 - 1, - 2*qr*qx - 2*qy*qz, 2*qx*(az - z) - 2*qz*(ax - x), 2*qy*(ax - x) - 4*qx*(ay - y) + 2*qr*(az - z),                 2*qx*(ax - x) + 2*qz*(az - z), 2*qy*(az - z) - 4*qz*(ay - y) - 2*qr*(ax - x)]
			//[ - 2*qr*qy - 2*qx*qz,   2*qr*qx - 2*qy*qz, 2*qx^2 + 2*qy^2 - 1, 2*qy*(ax - x) - 2*qx*(ay - y), 2*qz*(ax - x) - 2*qr*(ay - y) - 4*qx*(az - z), 2*qr*(ax - x) + 2*qz*(ay - y) - 4*qy*(az - z),                 2*qx*(ax - x) + 2*qy*(ay - y)]
			//
			const double qr = m_quat.r();
			const double qx = m_quat.x();
			const double qy = m_quat.y();
			const double qz = m_quat.z();


			MRPT_ALIGN16 const double vals1[3*7] = {
				2*qy2 + 2*qz2 - 1,
				-2*qr*qz - 2*qx*qy ,
				2*qr*qy - 2*qx*qz ,
				0,0,0,0,

 				2*qr*qz - 2*qx*qy ,
				2*qx2 + 2*qz2 - 1 ,
				-2*qr*qx - 2*qy*qz ,
				0,0,0,0,

				-2*qr*qy - 2*qx*qz ,
				2*qr*qx - 2*qy*qz ,
				2*qx2 + 2*qy2 - 1,
				0,0,0,0,
				};

			out_jacobian_df_dpose->loadFromArray(vals1);

			const double Ax = 2*(gx - m_coords[0]);
			const double Ay = 2*(gy - m_coords[1]);
			const double Az = 2*(gz - m_coords[2]);

			MRPT_ALIGN16 const double vals[3*4] = {
				-qy*Az + qz*Ay ,
				qy*Ay + qz*Az ,
				qx*Ay - 2*qy*Ax - qr*Az ,
				qx*Az + qr*Ay - 2*qz*Ax ,

				qx*Az - qz*Ax,
				qy*Ax - 2*qx*Ay + qr*Az,
				qx*Ax + qz*Az,
				qy*Az - 2*qz*Ay - qr*Ax,

				qy*Ax - qx*Ay,
				qz*Ax - qr*Ay - 2*qx*Az,
				qr*Ax + qz*Ay - 2*qy*Az,
				qx*Ax + qy*Ay
				};

			CMatrixDouble44  norm_jacob(UNINITIALIZED_MATRIX);
			this->quat().normalizationJacobian(norm_jacob);

			out_jacobian_df_dpose->insertMatrix(0,3, (CMatrixFixedNumeric<double,3,4>(vals)*norm_jacob).eval() );
		}
	}

	// function itself:
	m_quat.inverseRotatePoint(gx-m_coords[0],gy-m_coords[1],gz-m_coords[2],  lx,ly,lz);
}

/*---------------------------------------------------------------
	*=
  ---------------------------------------------------------------*/
void CPose3DQuat::operator *=(const double  s)
{
	m_coords[0]*=s;
	m_coords[1]*=s;
	m_coords[2]*=s;
	m_quat[0]*=s;
	m_quat[1]*=s;
	m_quat[2]*=s;
	m_quat[3]*=s;
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3DQuat::writeToStream(mrpt::utils::CStream &out,int *version) const
{
	if (version)
		*version = 0;
	else
	{
		out << m_coords[0] << m_coords[1] << m_coords[2] << m_quat[0] << m_quat[1] << m_quat[2] << m_quat[3];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3DQuat::readFromStream(mrpt::utils::CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			in >> m_coords[0] >> m_coords[1] >> m_coords[2] >> m_quat[0] >> m_quat[1] >> m_quat[2] >> m_quat[3];
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/*---------------------------------------------------------------
		sphericalCoordinates
---------------------------------------------------------------*/
void CPose3DQuat::sphericalCoordinates(
    const TPoint3D &point,
    double &out_range,
    double &out_yaw,
    double &out_pitch,
	mrpt::math::CMatrixFixedNumeric<double,3,3> *out_jacob_dryp_dpoint,
	mrpt::math::CMatrixFixedNumeric<double,3,7> *out_jacob_dryp_dpose
	) const
{
	const bool comp_jacobs = out_jacob_dryp_dpoint!=NULL || out_jacob_dryp_dpose!=NULL;

    // Pass to coordinates as seen from this 6D pose:
	CMatrixFixedNumeric<double,3,3> jacob_dinv_dpoint, *ptr_ja1 = comp_jacobs ? &jacob_dinv_dpoint : NULL;
	CMatrixFixedNumeric<double,3,7> jacob_dinv_dpose,  *ptr_ja2 = comp_jacobs ? &jacob_dinv_dpose : NULL;

	TPoint3D local;
	this->inverseComposePoint(point.x,point.y,point.z, local.x,local.y,local.z,ptr_ja1,ptr_ja2);

    // Range:
	out_range = local.norm();

    // Yaw:
    if (local.y!=0 || local.x!=0)
         out_yaw = atan2(local.y,local.x);
    else out_yaw = 0;

    // Pitch:
    if (out_range!=0)
         out_pitch = -asin( local.z / out_range );
    else out_pitch = 0;

	// Jacobians are:
	//  dryp_dpoint = dryp_dlocalpoint  * dinv_dpoint
	//  dryp_dpose  = dryp_dlocalpoint  * dinv_dpose
	if (comp_jacobs)
	{
		if (out_range==0)
			THROW_EXCEPTION("Jacobians are undefined for range=0")

		/* MATLAB:
			syms H h_range h_yaw h_pitch real;
			syms xi_ yi_ zi_ real;
			h_range = sqrt(xi_^2+yi_^2+zi_^2);
			h_yaw   = atan(yi_/xi_);
			% h_pitch = -asin(zi_/ sqrt( xi_^2 + yi_^2 + zi_^2 ) );
			h_pitch = -atan(zi_, sqrt( xi_^2 + yi_^2) );
			H=[ h_range ; h_yaw ; h_pitch ];
			jacob_fesf_xyz=jacobian(H,[xi_ yi_ zi_])
		*/
		const double _r = 1.0/out_range;
		const double x2 = square(local.x);
		const double y2 = square(local.y);

		const double t2 = std::sqrt(x2 + y2);
		const double _K = 1.0/(t2*square(out_range));

		double vals[3*3]= {
			local.x*_r,					local.y*_r,					local.z*_r,
			-local.y/(x2*(y2/x2 + 1)),	1.0/(local.x*(y2/x2 + 1)),  0,
			(local.x*local.z) * _K,		(local.y*local.z) * _K,		-t2/square(out_range)
			};

		const CMatrixDouble33 dryp_dlocalpoint(vals);
		if (out_jacob_dryp_dpoint)
			out_jacob_dryp_dpoint->multiply(dryp_dlocalpoint,jacob_dinv_dpoint);
		if (out_jacob_dryp_dpose)
			out_jacob_dryp_dpose->multiply(dryp_dlocalpoint,jacob_dinv_dpose);
	}
}


/**  Textual output stream function.
 */
std::ostream& mrpt::poses::operator << (std::ostream& o, const CPose3DQuat& p)
{
	const std::streamsize old_pre = o.precision();
	const ios_base::fmtflags old_flags = o.flags();
	o << "(x,y,z,qr,qx,qy,qz)=(" << std::fixed << std::setprecision(4)
		<< p.m_coords[0] << ","
		<< p.m_coords[1] << ","
		<< p.m_coords[2] << ","
		<< p.quat()[0] << ","
		<< p.quat()[1] << ","
		<< p.quat()[2] << ","
		<< p.quat()[3] << ")";
	o.flags(old_flags);
	o.precision(old_pre);
	return o;
}


/** Unary - operator: return the inverse pose "-p" (Note that is NOT the same than a pose with all its arguments multiplied by "-1") */
CPose3DQuat mrpt::poses::operator -(const CPose3DQuat &p)
{
	CPose3DQuat ret = p;
	ret.inverse();
	return ret;
}

/** Convert this pose into its inverse, saving the result in itself. \sa operator- */
void CPose3DQuat::inverse()
{
	// Invert translation:
	this->inverseComposePoint(
		0,0,0,
		m_coords[0],m_coords[1],m_coords[2]);

	// Invert rotation: [qr qx qy qz] ==> [qr -qx -qy -qz]
	m_quat[1] = -m_quat[1];
	m_quat[2] = -m_quat[2];
	m_quat[3] = -m_quat[3];
}

void CPose3DQuat::setToNaN()
{
	for (int i=0;i<3;i++)
		m_coords[i] = std::numeric_limits<double>::quiet_NaN();

	for (int i=0;i<4;i++)
		quat()[i] = std::numeric_limits<double>::quiet_NaN();
}
