/* +---------------------------------------------------------------------------+
   |          The Mobile Robot Programming Toolkit (MRPT) C++ library          |
   |                                                                           |
   |                   http://mrpt.sourceforge.net/                            |
   |                                                                           |
   |   Copyright (C) 2005-2010  University of Malaga                           |
   |                                                                           |
   |    This software was written by the Machine Perception and Intelligent    |
   |      Robotics Lab, University of Malaga (Spain).                          |
   |    Contact: Jose-Luis Blanco  <jlblanco@ctima.uma.es>                     |
   |                                                                           |
   |  This file is part of the MRPT project.                                   |
   |                                                                           |
   |     MRPT is free software: you can redistribute it and/or modify          |
   |     it under the terms of the GNU General Public License as published by  |
   |     the Free Software Foundation, either version 3 of the License, or     |
   |     (at your option) any later version.                                   |
   |                                                                           |
   |   MRPT is distributed in the hope that it will be useful,                 |
   |     but WITHOUT ANY WARRANTY; without even the implied warranty of        |
   |     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         |
   |     GNU General Public License for more details.                          |
   |                                                                           |
   |     You should have received a copy of the GNU General Public License     |
   |     along with MRPT.  If not, see <http://www.gnu.org/licenses/>.         |
   |                                                                           |
   +---------------------------------------------------------------------------+ */

#include <mrpt/base.h>  // Precompiled headers

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPoint2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPoint3D.h>

#include <mrpt/poses/CPose3DQuat.h>

#include <mrpt/math/utils.h>
#include <mrpt/math/CMatrix.h>


#ifndef M_SQRT1_2
#define M_SQRT1_2 0.70710678118654752440
#endif

using namespace mrpt;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;


IMPLEMENTS_SERIALIZABLE(CPose3D, CPose,mrpt::poses)

/*---------------------------------------------------------------
	Constructors
  ---------------------------------------------------------------*/
CPose3D::CPose3D()
	: m_ypr_uptodate(true), m_yaw(0),m_pitch(0),m_roll(0)
{
	m_is3D = true;
	m_x = 0;
	m_y = 0;
	m_z = 0;
	m_HM.unit();
}

CPose3D::CPose3D(const double x,const double y,const double z, const double yaw, const double pitch, const double roll)
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll()
{
	m_is3D = true;
	setFromValues(x,y,z,yaw,pitch,roll);
}

CPose3D::CPose3D( const CPose3D &o)
	: m_ypr_uptodate(o.m_ypr_uptodate), m_yaw(o.m_yaw),m_pitch(o.m_pitch),m_roll(o.m_roll), m_HM(o.m_HM)
{
	m_is3D = true;
	m_x = o.m_x;
	m_y = o.m_y;
	m_z = o.m_z;
}

CPose3D::CPose3D(const mrpt::math::TPose3D &o)
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll()
{
	m_is3D = true;
	setFromValues(o.x,o.y,o.z,o.yaw,o.pitch,o.roll);
}

CPose3D & CPose3D::operator =( const CPose3D &o)
{
	m_HM = o.m_HM;
	m_x = o.m_x;
	m_y = o.m_y;
	m_z = o.m_z;
	m_yaw = o.m_yaw;
	m_pitch = o.m_pitch;
	m_roll = o.m_roll;
	m_ypr_uptodate = o.m_ypr_uptodate;
	return *this;
}

CPose3D::CPose3D(const CPose2D &p)
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll()
{
	m_is3D = true;
	setFromValues(p.x(),p.y(),0, p.phi());
}

CPose3D::CPose3D(const CPoint3D &p)
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll()
{
	m_is3D = true;
	setFromValues(p.x(),p.y(),p.z());
}

CPose3D::CPose3D(const math::CMatrixDouble & m)
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll(),m_HM( CMatrixDouble44(m) )
{
	m_is3D = true;
	m_x = m_HM.get_unsafe(0,3);
	m_y = m_HM.get_unsafe(1,3);
	m_z = m_HM.get_unsafe(2,3);
	//updateYawPitchRoll();
}


CPose3D::CPose3D(const math::CMatrixDouble44 &m)
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll(),m_HM(m)
{
	m_is3D = true;
	m_x = m_HM.get_unsafe(0,3);
	m_y = m_HM.get_unsafe(1,3);
	m_z = m_HM.get_unsafe(2,3);
	//updateYawPitchRoll();
}

/** Constructor from a quaternion (which only represents the 3D rotation part) and a 3D displacement. */
CPose3D::CPose3D(const mrpt::math::CQuaternionDouble &q, const double _x, const double _y, const double _z )
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll(),m_HM( UNINITIALIZED_MATRIX )
{
	m_is3D = true;

	double yaw,pitch,roll;
	q.rpy(roll,pitch,yaw);
	this->setFromValues(_x,_y,_z,yaw,pitch,roll);
}

/** Constructor from a quaternion (which only represents the 3D rotation part) and a 3D displacement. */
CPose3D::CPose3D(const CPose3DQuat &p )
	: m_ypr_uptodate(false), m_yaw(),m_pitch(),m_roll(),m_HM( UNINITIALIZED_MATRIX )
{
	m_is3D = true;

	double yaw,pitch,roll;
	p.quat().rpy(roll,pitch,yaw);
	this->setFromValues(p.x(),p.y(),p.z(),yaw,pitch,roll);
}

/*---------------------------------------------------------------
   Implements the writing to a CStream capability of
     CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3D::writeToStream(CStream &out,int *version) const
{
	if (version)
		*version = 2;
	else
	{
		// Just for the case the user has modified by hand (x,y,z,yaw,pitch,roll) directly:
		const_cast<CPose3D*>(this)->rebuildHomogeneousMatrix();
		const CPose3DQuat  q(*this);
		// The coordinates:
		out << q[0] << q[1] << q[2] << q[3] << q[4] << q[5] << q[6];
	}
}

/*---------------------------------------------------------------
	Implements the reading from a CStream capability of
		CSerializable objects
  ---------------------------------------------------------------*/
void  CPose3D::readFromStream(CStream &in,int version)
{
	switch(version)
	{
	case 0:
		{
			// The coordinates:
			CMatrix  HM2;
			in >> HM2;

			m_HM = HM2;

			m_x = m_HM.get_unsafe(0,3);
			m_y = m_HM.get_unsafe(1,3);
			m_z = m_HM.get_unsafe(2,3);
			updateYawPitchRoll();
		} break;
	case 1:
		{
			// The coordinates:
			in >> m_HM;

			m_x = m_HM.get_unsafe(0,3);
			m_y = m_HM.get_unsafe(1,3);
			m_z = m_HM.get_unsafe(2,3);
			updateYawPitchRoll();
		} break;
	case 2:
		{
			// An equivalent CPose3DQuat
			CPose3DQuat p(UNINITIALIZED_QUATERNION);
			in >>p[0]>>p[1]>>p[2]>>p[3]>>p[4]>>p[5]>>p[6];

			double _yaw,_pitch,_roll;
			p.quat().rpy(_roll,_pitch,_yaw);
			this->setFromValues(p.x(),p.y(),p.z(),_yaw,_pitch,_roll);
		} break;
	default:
		MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version)

	};
}

/**  Textual output stream function.
 */
std::ostream& mrpt::poses::operator << (std::ostream& o, const CPose3D& p)
{
	p.updateYawPitchRoll();
	o << "(x,y,z,yaw,pitch,roll)=(" << std::fixed << std::setprecision(4) << p.m_x << "," << p.m_y << "," << p.m_z <<  ","
		<< std::setprecision(2) << RAD2DEG(p.m_yaw) << "deg," << RAD2DEG(p.m_pitch) << "deg," << RAD2DEG(p.m_roll) << "deg)";
	return o;
}

/*---------------------------------------------------------------
				normalizeAngles
---------------------------------------------------------------*/
void  CPose3D::normalizeAngles()
{
	updateYawPitchRoll();
	//setFromValues(m_x,m_y,m_z,m_yaw,m_pitch,m_roll);
}


/*---------------------------------------------------------------
 Set the pose from 3D point and yaw/pitch/roll angles, in radians.
---------------------------------------------------------------*/
void  CPose3D::setFromValues(
	const double		x0,
	const double		y0,
	const double		z0,
	const double		yaw,
	const double		pitch,
	const double		roll)
{
	m_x = x0;
	m_y = y0;
	m_z = z0;
	this->m_yaw = mrpt::math::wrapToPi(yaw);
	this->m_pitch = mrpt::math::wrapToPi(pitch);
	this->m_roll = mrpt::math::wrapToPi(roll);

	m_ypr_uptodate = true;

	rebuildHomogeneousMatrix();
}

/*---------------------------------------------------------------
 Set the pose from 3D point and yaw/pitch/roll angles, in radians.
---------------------------------------------------------------*/
void  CPose3D::rebuildHomogeneousMatrix()
{
	m_HM.get_unsafe(0,3)=m_x;
	m_HM.get_unsafe(1,3)=m_y;
	m_HM.get_unsafe(2,3)=m_z;

	m_HM.get_unsafe(3,0)=
	m_HM.get_unsafe(3,1)=
	m_HM.get_unsafe(3,2)=0;
	m_HM.get_unsafe(3,3)=1.0;

#ifdef HAVE_SINCOS
	double	cy,sy;
	::sincos(m_yaw,&sy,&cy);
	double	cp,sp;
	::sincos(m_pitch,&sp,&cp);
	double	cr,sr;
	::sincos(m_roll,&sr,&cr);
#else
	const double	cy = cos(m_yaw);
	const double	sy = sin(m_yaw);
	const double	cp = cos(m_pitch);
	const double	sp = sin(m_pitch);
	const double	cr = cos(m_roll);
	const double	sr = sin(m_roll);
#endif

	m_HM.get_unsafe(0,0)= cy*cp;	m_HM.get_unsafe(0,1)= cy*sp*sr-sy*cr;		m_HM.get_unsafe(0,2)= cy*sp*cr+sy*sr;
	m_HM.get_unsafe(1,0)= sy*cp;	m_HM.get_unsafe(1,1)= sy*sp*sr+cy*cr;		m_HM.get_unsafe(1,2)= sy*sp*cr-cy*sr;
	m_HM.get_unsafe(2,0)= -sp;		m_HM.get_unsafe(2,1)= cp*sr;				m_HM.get_unsafe(2,2)= cp*cr;
}


/*---------------------------------------------------------------
		Scalar multiplication.
---------------------------------------------------------------*/
void CPose3D::operator *=(const double s)
{
	updateYawPitchRoll();
	m_x*=s;
	m_y*=s;
	m_z*=s;
	m_yaw*=s;
	m_pitch*=s;
	m_roll*=s;
	setFromValues(m_x,m_y,m_z,m_yaw,m_pitch,m_roll);
}

/*---------------------------------------------------------------
		getYawPitchRoll
---------------------------------------------------------------*/
void  CPose3D::getYawPitchRoll( double &yaw, double &pitch, double &roll ) const
{
	ASSERTDEBMSG_( fabs(sqrt(square(m_HM(0,0))+square(m_HM(1,0))+square(m_HM(2,0))) - 1 ) < 1e-5, "Homogeneous matrix is not orthogonal & normalized!: "+m_HM.inMatlabFormat() )
	ASSERTDEBMSG_( fabs(sqrt(square(m_HM(0,1))+square(m_HM(1,1))+square(m_HM(2,1))) - 1 ) < 1e-5, "Homogeneous matrix is not orthogonal & normalized!: "+m_HM.inMatlabFormat() )
	ASSERTDEBMSG_( fabs(sqrt(square(m_HM(0,2))+square(m_HM(1,2))+square(m_HM(2,2))) - 1 ) < 1e-5, "Homogeneous matrix is not orthogonal & normalized!: "+m_HM.inMatlabFormat() )

	// Pitch is in the range [-pi/2, pi/2 ], so this calculation is enough:
	pitch =  atan2( - m_HM.get_unsafe(2,0), hypot( m_HM.get_unsafe(0,0),m_HM.get_unsafe(1,0) ) ); //asin( - m_HM.get_unsafe(2,0) );

	// Roll:
	if ( (fabs(m_HM.get_unsafe(2,1))+fabs(m_HM.get_unsafe(2,2)))<10*std::numeric_limits<double>::epsilon() )
	{
		//Gimbal lock between yaw and roll. This one is arbitrarily forced to be zero.
		//Check http://reference.mrpt.org/svn/classmrpt_1_1poses_1_1_c_pose3_d.html. If cos(pitch)==0, the homogeneous matrix is:
		//When sin(pitch)==1:
		//  /0  cysr-sycr cycr+sysr x\   /0  sin(r-y) cos(r-y)  x\.
		//  |0  sysr+cycr sycr-cysr y| = |0  cos(r-y) -sin(r-y) y|
		//  |-1     0         0     z|   |-1    0         0     z|
		//  \0      0         0     1/   \0     0         0     1/
		//
		//And when sin(pitch)=-1:
		//  /0 -cysr-sycr -cycr+sysr x\   /0 -sin(r+y) -cos(r+y) x\.
		//  |0 -sysr+cycr -sycr-cysr y| = |0 cos(r+y)  -sin(r+y) y|
		//  |1      0          0     z|   |1    0          0     z|
		//  \0      0          0     1/   \0    0          0     1/
		//
		//Both cases are in a "gimbal lock" status. This happens because pitch is vertical.

		roll = 0.0;
		if (pitch>0) yaw=atan2(m_HM.get_unsafe(1,2),m_HM.get_unsafe(0,2));
		else yaw=atan2(-m_HM.get_unsafe(1,2),-m_HM.get_unsafe(0,2));
	}
	else
	{
		roll = atan2( m_HM.get_unsafe(2,1), m_HM.get_unsafe(2,2) );
		// Yaw:
		yaw = atan2( m_HM.get_unsafe(1,0), m_HM.get_unsafe(0,0) );
	}
}


/*---------------------------------------------------------------
		sphericalCoordinates
---------------------------------------------------------------*/
void CPose3D::sphericalCoordinates(
    const TPoint3D &point,
    double &out_range,
    double &out_yaw,
    double &out_pitch ) const
{
    // Pass to coordinates as seen from this 6D pose:
	TPoint3D local;
	this->inverseComposePoint(point.x,point.y,point.z, local.x,local.y,local.z);

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

}


/*---------------------------------------------------------------
		addComponents
---------------------------------------------------------------*/
void CPose3D::addComponents(const CPose3D &p)
{
	updateYawPitchRoll();
	m_x+=p.m_x;
	m_y+=p.m_y;
	m_z+=p.m_z;
	m_yaw+=p.m_yaw;
	m_pitch+=p.m_pitch;
	m_roll+=p.m_roll;
}


/*---------------------------------------------------------------
		distanceEuclidean6D
---------------------------------------------------------------*/
double CPose3D::distanceEuclidean6D( const CPose3D &o ) const
{
	updateYawPitchRoll();
	o.updateYawPitchRoll();
	return sqrt(
		square( o.m_x - m_x ) +
		square( o.m_y - m_y ) +
		square( o.m_z - m_z ) +
		square( wrapToPi( o.m_yaw - m_yaw ) ) +
		square( wrapToPi( o.m_pitch - m_pitch ) ) +
		square( wrapToPi( o.m_roll - m_roll ) ) );
}


/*---------------------------------------------------------------
		composePoint
---------------------------------------------------------------*/
void CPose3D::composePoint(double lx,double ly,double lz, double &gx, double &gy, double &gz,
	mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose,
	mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dse3,
	bool use_small_rot_approx ) const
{
	// Jacob: df/dpoint
	if (out_jacobian_df_dpoint)
	{
		m_HM.extractMatrix(0,0, *out_jacobian_df_dpoint);
	}

	// Jacob: df/dpose
	if (out_jacobian_df_dpose)
	{
		if (use_small_rot_approx)
		{
			// Linearized Jacobians around (yaw,pitch,roll)=(0,0,0):
			const double nums[3*6] = {
				1, 0, 0, -ly, lz,   0,
				0, 1, 0,  lx,  0, -lz,
				0, 0, 1,   0, -lx,  ly };
			out_jacobian_df_dpose->loadFromArray(nums);
		}
		else
		{
			// Exact Jacobians:
			updateYawPitchRoll();
	#	ifdef HAVE_SINCOS
			double	cy,sy;
			::sincos(m_yaw,&sy,&cy);
			double	cp,sp;
			::sincos(m_pitch,&sp,&cp);
			double	cr,sr;
			::sincos(m_roll,&sr,&cr);
	#	else
			const double	cy = cos(m_yaw);
			const double	sy = sin(m_yaw);
			const double	cp = cos(m_pitch);
			const double	sp = sin(m_pitch);
			const double	cr = cos(m_roll);
			const double	sr = sin(m_roll);
	#	endif

			const double nums[3*6] = {
				1, 0, 0,
					-lx*sy*cp+ly*(-sy*sp*sr-cy*cr)+lz*(-sy*sp*cr+cy*sr),   // d_x'/d_yaw
					-lx*cy*sp+ly*(cy*cp*sr       )+lz*(cy*cp*cr      ),   // d_x'/d_pitch
							  ly*(cy*sp*cr+sy*sr)+lz*(-cy*sp*sr+sy*cr),   // d_x'/d_roll
				0, 1, 0,
					 lx*cy*cp+ly*(cy*sp*sr-sy*cr)+lz*(cy*sp*cr+sy*sr),   // d_y'/d_yaw
					-lx*sy*sp+ly*(sy*cp*sr)      +lz*(sy*cp*cr      ),   // d_y'/d_pitch
							  ly*(sy*sp*cr-cy*sr)+lz*(-sy*sp*sr-cy*cr),   // d_y'/d_roll
				0, 0, 1,
					0,  // d_z' / d_yaw
					-lx*cp-ly*sp*sr-lz*sp*cr,  // d_z' / d_pitch
					ly*cp*cr-lz*cp*sr  // d_z' / d_roll
				};
			out_jacobian_df_dpose->loadFromArray(nums);
		}
	}

	gx=m_HM.get_unsafe(0,0)*lx+m_HM.get_unsafe(0,1)*ly+m_HM.get_unsafe(0,2)*lz+m_x;
	gy=m_HM.get_unsafe(1,0)*lx+m_HM.get_unsafe(1,1)*ly+m_HM.get_unsafe(1,2)*lz+m_y;
	gz=m_HM.get_unsafe(2,0)*lx+m_HM.get_unsafe(2,1)*ly+m_HM.get_unsafe(2,2)*lz+m_z;

	// Jacob: df/dse3
	if (out_jacobian_df_dse3)
	{
		const double nums[3*6] = {
			1, 0, 0,   0, gz, -gy,
			0, 1, 0, -gz,  0,  gx,
			0, 0, 1,  gy,-gx,   0 };
		out_jacobian_df_dse3->loadFromArray(nums);
	}
}


// TODO: Use SSE2? OTOH, this forces mem align...
#if MRPT_HAS_SSE2 && defined(MRPT_USE_SSE2)
/*static inline __m128 transformSSE(const __m128* matrix, const __m128& in)
{
    ASSERT_(((size_t)matrix & 15) == 0);
    __m128 a0 = _mm_mul_ps(_mm_load_ps((float*)(matrix+0)), _mm_shuffle_ps(in,in,_MM_SHUFFLE(0,0,0,0)));
    __m128 a1 = _mm_mul_ps(_mm_load_ps((float*)(matrix+1)), _mm_shuffle_ps(in,in,_MM_SHUFFLE(1,1,1,1)));
    __m128 a2 = _mm_mul_ps(_mm_load_ps((float*)(matrix+2)), _mm_shuffle_ps(in,in,_MM_SHUFFLE(2,2,2,2)));

    return _mm_add_ps(_mm_add_ps(a0,a1),a2);
}*/
#endif // SSE2

/*---------------------------------------------------------------
		getAsVector
---------------------------------------------------------------*/
void CPose3D::getAsVector(vector_double &r) const
{
	updateYawPitchRoll();
	r.resize(6);
	r[0]=m_x;
	r[1]=m_y;
	r[2]=m_z;
	r[3]=m_yaw;
	r[4]=m_pitch;
	r[5]=m_roll;
}


/*---------------------------------------------------------------
		unary -
---------------------------------------------------------------*/
CPose3D mrpt::poses::operator -(const CPose3D &b)
{
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	b.getInverseHomogeneousMatrix( B_INV );
	return CPose3D(B_INV);
}

/*---------------------------------------------------------------
		getAsQuaternion
---------------------------------------------------------------*/
void CPose3D::getAsQuaternion(mrpt::math::CQuaternionDouble &q, mrpt::math::CMatrixFixedNumeric<double,4,3>   *out_dq_dr ) const
{
	updateYawPitchRoll();
	// See: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	const double	cy = cos(m_yaw*0.5);
	const double	sy = sin(m_yaw*0.5);
	const double	cp = cos(m_pitch*0.5);
	const double	sp = sin(m_pitch*0.5);
	const double	cr = cos(m_roll*0.5);
	const double	sr = sin(m_roll*0.5);

	const double ccc = cr*cp*cy;
	const double ccs = cr*cp*sy;
	const double css = cr*sp*sy;
	const double sss = sr*sp*sy;
	const double scc = sr*cp*cy;
	const double ssc = sr*sp*cy;
	const double csc = cr*sp*cy;
	const double scs = sr*cp*sy;

	q[0] = ccc+sss;
	q[1] = scc-css;
	q[2] = csc+scs;
	q[3] = ccs-ssc;

	// Compute 4x3 Jacobian: for details, see technical report:
	//   Parameterizations of SE(3) transformations: equivalences, compositions and uncertainty, J.L. Blanco (2010).
	//   http://www.mrpt.org/6D_poses:equivalences_compositions_and_uncertainty
	if (out_dq_dr)
	{
		const double nums[4*3] = {
			-0.5*q[3], 0.5*( -csc+scs ), -0.5*q[1],
			-0.5*q[2],  0.5*( -ssc-ccs ), 0.5* q[0],
			0.5*q[1], 0.5*( ccc-sss  ),  0.5*q[3],
			0.5* q[0], 0.5*( -css-scc ), -0.5*q[2]
		};
		out_dq_dr->loadFromArray(nums);
	}
}


bool mrpt::poses::operator==(const CPose3D &p1,const CPose3D &p2)
{
	return (p1.x()==p2.x())&&(p1.y()==p2.y())&&(p1.z()==p2.z())&&(p1.yaw()==p2.yaw())&&(p1.pitch()==p2.pitch())&&(p1.roll()==p2.roll());
}

bool mrpt::poses::operator!=(const CPose3D &p1,const CPose3D &p2)
{
	return (p1.x()!=p2.x())||(p1.y()!=p2.y())||(p1.z()!=p2.z())||(p1.yaw()!=p2.yaw())||(p1.pitch()!=p2.pitch())||(p1.roll()!=p2.roll());
}


/*---------------------------------------------------------------
				point3D = pose3D + point3D
  ---------------------------------------------------------------*/
CPoint3D  CPose3D::operator + (const CPoint3D& b) const
{
	return CPoint3D(
		m_HM.get_unsafe(0,0)*b.x() + m_HM.get_unsafe(0,1)*b.y() +m_HM.get_unsafe(0,2)*b.z() + m_HM.get_unsafe(0,3),
		m_HM.get_unsafe(1,0)*b.x() + m_HM.get_unsafe(1,1)*b.y() +m_HM.get_unsafe(1,2)*b.z() + m_HM.get_unsafe(1,3),
		m_HM.get_unsafe(2,0)*b.x() + m_HM.get_unsafe(2,1)*b.y() +m_HM.get_unsafe(2,2)*b.z() + m_HM.get_unsafe(2,3) );
}

/*---------------------------------------------------------------
				point3D = pose3D + point2D
  ---------------------------------------------------------------*/
CPoint3D  CPose3D::operator + (const CPoint2D& b) const
{
	return CPoint3D(
		m_HM.get_unsafe(0,0)*b.x() + m_HM.get_unsafe(0,1)*b.y() + m_HM.get_unsafe(0,3),
		m_HM.get_unsafe(1,0)*b.x() + m_HM.get_unsafe(1,1)*b.y() + m_HM.get_unsafe(1,3),
		m_HM.get_unsafe(2,0)*b.x() + m_HM.get_unsafe(2,1)*b.y() + m_HM.get_unsafe(2,3) );
}


/*---------------------------------------------------------------
				this = A + B
  ---------------------------------------------------------------*/
void CPose3D::composeFrom(const CPose3D& A, const CPose3D& B )
{
	//m_HM.multiply( A.m_HM, B.m_HM );
	// In fact, we can just update the 3x4 part of the matrix:
	for (int r=0;r<3;r++)
		for (int c=0;c<3;c++)
			m_HM.get_unsafe(r,c) = A.m_HM.get_unsafe(r,0)*B.m_HM.get_unsafe(0,c)+A.m_HM.get_unsafe(r,1)*B.m_HM.get_unsafe(1,c)+A.m_HM.get_unsafe(r,2)*B.m_HM.get_unsafe(2,c);

	// The translation part M(0:3,3)
	for (int r=0;r<3;r++)
			m_HM.get_unsafe(r,3) = A.m_HM.get_unsafe(r,0)*B.m_HM.get_unsafe(0,3)+A.m_HM.get_unsafe(r,1)*B.m_HM.get_unsafe(1,3)+A.m_HM.get_unsafe(r,2)*B.m_HM.get_unsafe(2,3)+ A.m_HM.get_unsafe(r,3);

	// Update the x,y,z,yaw,pitch,roll fields:
	m_x = m_HM.get_unsafe(0,3);
	m_y = m_HM.get_unsafe(1,3);
	m_z = m_HM.get_unsafe(2,3);
	m_ypr_uptodate=false;
}

/** Convert this pose into its inverse, saving the result in itself. */
void CPose3D::inverse()
{
	mrpt::math::homogeneousMatrixInverse(m_HM); // Inverse of the HM.

	// Update the x,y,z,yaw,pitch,roll fields:
	m_x = m_HM.get_unsafe(0,3);
	m_y = m_HM.get_unsafe(1,3);
	m_z = m_HM.get_unsafe(2,3);
	m_ypr_uptodate=false;
}

/*---------------------------------------------------------------
						isHorizontal
 ---------------------------------------------------------------*/
bool CPose3D::isHorizontal( const double tolerance  ) const
{
	return (fabs(m_pitch)<=tolerance) &&
	       ( fabs(m_roll)<=tolerance || fabs(mrpt::math::wrapToPi( m_roll-M_PI))<=tolerance );
}


/**  Makes \f$ this = A \ominus B \f$ this method is slightly more efficient than "this= A - B;" since it avoids the temporary object.
  *  \note A or B can be "this" without problems.
  * \sa composeFrom, composePoint
  */
void CPose3D::inverseComposeFrom(const CPose3D& A, const CPose3D& B )
{
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	B.getInverseHomogeneousMatrix( B_INV );

	m_HM.multiply(B_INV,A.m_HM);

	m_x = m_HM.get_unsafe(0,3);
	m_y = m_HM.get_unsafe(1,3);
	m_z = m_HM.get_unsafe(2,3);
	m_ypr_uptodate=false;
}

/**  Computes the 3D point L such as \f$ L = G \ominus this \f$.
  * \sa composePoint, composeFrom
  */
void CPose3D::inverseComposePoint(const double gx,const double gy,const double gz,double &lx,double &ly,double &lz,
	mrpt::math::CMatrixFixedNumeric<double,3,3>  *out_jacobian_df_dpoint,
	mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dpose,
	mrpt::math::CMatrixFixedNumeric<double,3,6>  *out_jacobian_df_dse3
	)const
{
	CMatrixDouble44 B_INV(UNINITIALIZED_MATRIX);
	getInverseHomogeneousMatrix( B_INV );

	// Jacob: df/dpoint
	if (out_jacobian_df_dpoint)
	{
		B_INV.extractMatrix(0,0, *out_jacobian_df_dpoint);
	}

	// Jacob: df/dpose
	if (out_jacobian_df_dpose)
	{
		updateYawPitchRoll();

#	ifdef HAVE_SINCOS
		double	cy,sy;
		::sincos(m_yaw,&sy,&cy);
		double	cp,sp;
		::sincos(m_pitch,&sp,&cp);
		double	cr,sr;
		::sincos(m_roll,&sr,&cr);
#	else
		const double	cy = cos(m_yaw);
		const double	sy = sin(m_yaw);
		const double	cp = cos(m_pitch);
		const double	sp = sin(m_pitch);
		const double	cr = cos(m_roll);
		const double	sr = sin(m_roll);
#	endif

		const double m11_dy = -sy*cp;  	const double m12_dy = cy*cp; 	const double m13_dy = 0;
		const double m11_dp = -cy*sp;  	const double m12_dp = -sy*sp; 	const double m13_dp = -cp;
		const double m11_dr = 0;  		const double m12_dr = 0;  		const double m13_dr = 0;

		const double m21_dy = (-sy*sp*sr-cy*cr);  	const double m22_dy = (cy*sp*sr-sy*cr); 	const double m23_dy = 0;
		const double m21_dp = (cy*cp*sr );  		const double m22_dp = (sy*cp*sr ); 			const double m23_dp = -sp*sr;
		const double m21_dr = (cy*sp*cr+sy*sr);  	const double m22_dr = (sy*sp*cr-cy*sr);  	const double m23_dr = cp*cr;

		const double m31_dy = (-sy*sp*cr+cy*sr);  	const double m32_dy = (cy*sp*cr+sy*sr); 	const double m33_dy = 0;
		const double m31_dp = (cy*cp*cr );  		const double m32_dp = (sy*cp*cr ); 			const double m33_dp = -sp*cr;
		const double m31_dr = (-cy*sp*sr+sy*cr);  	const double m32_dr = (-sy*sp*sr-cy*cr);  	const double m33_dr = -cp*sr;

		const double Ax = gx-m_x;
		const double Ay = gy-m_y;
		const double Az = gz-m_z;

		const double nums[3*6] = {
			-m_HM.get_unsafe(0,0), -m_HM.get_unsafe(1,0), -m_HM.get_unsafe(2,0),
				Ax*m11_dy + Ay*m12_dy + Az*m13_dy ,  // d_x'/d_yaw
				Ax*m11_dp + Ay*m12_dp + Az*m13_dp,  // d_x'/d_pitch
				Ax*m11_dr + Ay*m12_dr + Az*m13_dr,  // d_x'/d_roll

			-m_HM.get_unsafe(0,1), -m_HM.get_unsafe(1,1), -m_HM.get_unsafe(2,1),
				Ax*m21_dy + Ay*m22_dy + Az*m23_dy,  // d_x'/d_yaw
				Ax*m21_dp + Ay*m22_dp + Az*m23_dp,  // d_x'/d_pitch
				Ax*m21_dr + Ay*m22_dr + Az*m23_dr,  // d_x'/d_roll

			-m_HM.get_unsafe(0,2), -m_HM.get_unsafe(1,2), -m_HM.get_unsafe(2,2),
				Ax*m31_dy + Ay*m32_dy + Az*m33_dy,  // d_x'/d_yaw
				Ax*m31_dp + Ay*m32_dp + Az*m33_dp,  // d_x'/d_pitch
				Ax*m31_dr + Ay*m32_dr + Az*m33_dr,  // d_x'/d_roll
			};
		out_jacobian_df_dpose->loadFromArray(nums);
	}

	lx = B_INV.get_unsafe(0,0) * gx + B_INV.get_unsafe(0,1) * gy + B_INV.get_unsafe(0,2) * gz + B_INV.get_unsafe(0,3);
	ly = B_INV.get_unsafe(1,0) * gx + B_INV.get_unsafe(1,1) * gy + B_INV.get_unsafe(1,2) * gz + B_INV.get_unsafe(1,3);
	lz = B_INV.get_unsafe(2,0) * gx + B_INV.get_unsafe(2,1) * gy + B_INV.get_unsafe(2,2) * gz + B_INV.get_unsafe(2,3);

	// Jacob: df/dse3
	if (out_jacobian_df_dse3)
	{
		const double nums[3*6] = {
			-1, 0, 0,   0,-lz,  ly,
			0, -1, 0,  lz,  0, -lx,
			0, 0, -1, -ly, lx,   0 };
		out_jacobian_df_dse3->loadFromArray(nums);
	}
}


/** Exponentiate a Vector in the SE3 Lie Algebra to generate a new CPose3D.
  * \note Method from TooN (C) Tom Drummond (GNU GPL)
  */
CPose3D CPose3D::exp(const mrpt::math::CArrayNumeric<double,6> & mu)
{
	static const double one_6th = 1.0/6.0;
	static const double one_20th = 1.0/20.0;

	CMatrixDouble44 HM; // For the result
	HM(3,3)=1;

	const CArrayDouble<3> mu_xyz = CArrayDouble<3>( mu.slice<0,3>() );
	const CArrayDouble<3> w = CArrayDouble<3>( mu.slice<3,3>() );
	const double theta_sq = w.squareNorm(); // CArrayDouble<3>(w).Square().sumAll(); // w*w;
	const double theta = sqrt(theta_sq);
	double A, B;

	CArrayDouble<3> cross;
	mrpt::math::crossProduct3D(w, mu_xyz, cross );

	if (theta_sq < 1e-8) {
		A = 1.0 - one_6th * theta_sq;
		B = 0.5;
		HM(0,3) = mu_xyz[0] + 0.5 * cross[0];
		HM(1,3) = mu_xyz[1] + 0.5 * cross[1];
		HM(2,3) = mu_xyz[2] + 0.5 * cross[2];
	} else {
		double C;
		if (theta_sq < 1e-6) {
			C = one_6th*(1.0 - one_20th * theta_sq);
			A = 1.0 - theta_sq * C;
			B = 0.5 - 0.25 * one_6th * theta_sq;
		} else {
			const double inv_theta = 1.0/theta;
			A = sin(theta) * inv_theta;
			B = (1 - cos(theta)) * (inv_theta * inv_theta);
			C = (1 - A) * (inv_theta * inv_theta);
		}

		CArrayDouble<3> w_cross;	// = w^cross
		mrpt::math::crossProduct3D(w, cross, w_cross );

		HM(0,3) = mu_xyz[0] + B * cross[0] + C * w_cross[0];
		HM(1,3) = mu_xyz[1] + B * cross[1] + C * w_cross[1];
		HM(2,3) = mu_xyz[2] + B * cross[2] + C * w_cross[2];
		//result.get_translation() = mu_xyz + B * cross + C * (w ^ cross);
	}

	// Write 3x3 rotation part in a submatrix view of HM:
	CSubmatrixView<CMatrixDouble44,3,3> rot_part(HM,0,0);
	mrpt::math::rodrigues_so3_exp(w, A, B, rot_part);

	return CPose3D(HM);
}


/** Take the logarithm of the 3x3 rotation matrix, generating the corresponding vector in the Lie Algebra.
  * \note Method from TooN (C) Tom Drummond (GNU GPL)
  */
CArrayDouble<3> CPose3D::ln_rotation() const
{
	CArrayDouble<3> result;

	const double cos_angle = (m_HM(0,0)+m_HM(1,1)+m_HM(2,2)- 1.0) * 0.5;
	result[0] = (m_HM(2,1)-m_HM(1,2))*0.5;
	result[1] = (m_HM(0,2)-m_HM(2,0))*0.5;
	result[2] = (m_HM(1,0)-m_HM(0,1))*0.5;

	double sin_angle_abs = result.norm(); //sqrt(result*result);
	if (cos_angle > M_SQRT1_2)
	{            // [0 - Pi/4[ use asin
		if(sin_angle_abs > 0){
			result *= asin(sin_angle_abs) / sin_angle_abs;
		}
	}
	else if( cos_angle > -M_SQRT1_2)
	{    // [Pi/4 - 3Pi/4[ use acos, but antisymmetric part
		double angle = acos(cos_angle);
		result *= angle / sin_angle_abs;
	}
	else
	{  // rest use symmetric part
		// antisymmetric part vanishes, but still large rotation, need information from symmetric part
		const double angle = M_PI - asin(sin_angle_abs);
		const double d0 = m_HM(0,0) - cos_angle,
			d1 = m_HM(1,1) - cos_angle,
			d2 = m_HM(2,2) - cos_angle;
		CArrayDouble<3> r2;
		if(fabs(d0) > fabs(d1) && fabs(d0) > fabs(d2))
		{ // first is largest, fill with first column
			r2[0] = d0;
			r2[1] = (m_HM(1,0)+m_HM(0,1))/2;
			r2[2] = (m_HM(0,2)+m_HM(2,0))/2;
		}
		else if(fabs(d1) > fabs(d2))
		{ 			    // second is largest, fill with second column
			r2[0] = (m_HM(1,0)+m_HM(0,1))/2;
			r2[1] = d1;
			r2[2] = (m_HM(2,1)+m_HM(1,2))/2;
		}
		else
		{							    // third is largest, fill with third column
			r2[0] = (m_HM(0,2)+m_HM(2,0))/2;
			r2[1] = (m_HM(2,1)+m_HM(1,2))/2;
			r2[2] = d2;
		}
		// flip, if we point in the wrong direction!
		if( (r2 * result).sumAll() < 0)
			r2 *= -1;
		//r2 *= 1.0/r2.norm();
		//result = angle * r2;
		result = r2;
		result *= (angle/r2.norm());
	}
	return result;
}

/** Exponentiate a vector in the Lie algebra to generate a new SO3 (a 3x3 rotation matrix).
  * \note Method from TooN (C) Tom Drummond (GNU GPL) */
CMatrixDouble33 CPose3D::exp_rotation(const mrpt::math::CArrayNumeric<double,3> & w)
{
	using std::sqrt;
	using std::sin;
	using std::cos;

	static const double one_6th = 1.0/6.0;
	static const double one_20th = 1.0/20.0;

	CMatrixDouble33 result;

	const double theta_sq = w.squareNorm(); //w*w;
	const double theta = sqrt(theta_sq);
	double A, B;
	//Use a Taylor series expansion near zero. This is required for
	//accuracy, since sin t / t and (1-cos t)/t^2 are both 0/0.
	if (theta_sq < 1e-8) {
		A = 1.0 - one_6th * theta_sq;
		B = 0.5;
	} else {
		if (theta_sq < 1e-6) {
			B = 0.5 - 0.25 * one_6th * theta_sq;
			A = 1.0 - theta_sq * one_6th*(1.0 - one_20th * theta_sq);
		} else {
			const double inv_theta = 1.0/theta;
			A = sin(theta) * inv_theta;
			B = (1 - cos(theta)) * (inv_theta * inv_theta);
		}
	}
	mrpt::math::rodrigues_so3_exp(w, A, B, result);
	return result;
}

/** Take the logarithm of the 3x4 matrix defined by this pose, generating the corresponding vector in the SE3 Lie Algebra.
  * \note Method from TooN (C) Tom Drummond (GNU GPL)
  */
CArrayDouble<6> CPose3D::ln() const
{
	CArrayDouble<3> rot = this->ln_rotation();
	const double theta =  rot.norm(); //sqrt(rot*rot);

	double shtot = 0.5;
	if(theta > 0.00001)
		shtot = sin(theta*0.5)/theta;

	// now do the rotation
	CArrayDouble<3> rot_half = rot;
	rot_half*=-0.5;
	const CMatrixDouble33 halfrotator = CPose3D::exp_rotation(rot_half);

	CArrayDouble<3> t;
	t[0] = m_x;
	t[1] = m_y;
	t[2] = m_z;

	CArrayDouble<3> rottrans; // = halfrotator * t;
	halfrotator.multiply_Ab(t, rottrans);

	if(theta > 0.001)
	{
		rottrans -= rot * ( (t * rot).sumAll() * (1-2*shtot) / rot.squareNorm() ); //(rot*rot));
	}
	else
	{
		rottrans -= rot * ((t * rot).sumAll()/24);
	}

	rottrans *= 1.0/(2 * shtot);

	CArrayDouble<6> result;
	::memcpy(&result[0], &rottrans[0], 3*sizeof(result[0]) );
	::memcpy(&result[3], &rot[0], 3*sizeof(result[0]) );
	return result;
}
